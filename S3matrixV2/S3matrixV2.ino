#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <FastLED.h>

// ===================== MATRIX =====================
#define LED_PIN     14
#define NUM_LEDS    64
#define W           8
#define H           8
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB

// cap brightness ~60%
#define MAX_BOARD_BRIGHTNESS 153

CRGB leds[NUM_LEDS];

// Your wiring: rows restart, bottom row starts on right and goes left
uint16_t XY(uint8_t x, uint8_t y) {
  uint8_t yy = (H - 1) - y;   // bottom row first
  uint8_t xx = (W - 1) - x;   // right -> left
  return (yy * W) + xx;
}
inline void drawPixel(int x, int y, const CRGB &c) {
  if (x < 0 || x >= W || y < 0 || y >= H) return;
  leds[XY((uint8_t)x, (uint8_t)y)] = c;
}
inline void applyBrightnessCap(uint8_t b = MAX_BOARD_BRIGHTNESS) {
  FastLED.setBrightness(min<uint8_t>(b, MAX_BOARD_BRIGHTNESS));
}

// ===================== QMI8658 (I2C) =====================
#define I2C_SDA 11
#define I2C_SCL 12

static const uint8_t REG_WHOAMI = 0x00;
static const uint8_t REG_CTRL1  = 0x02;
static const uint8_t REG_CTRL2  = 0x03;
static const uint8_t REG_CTRL3  = 0x04;
static const uint8_t REG_CTRL5  = 0x06;
static const uint8_t REG_CTRL7  = 0x08;
static const uint8_t REG_AX_L   = 0x35;

uint8_t imuAddr = 0;
bool imuOK = false;

bool writeReg(uint8_t a, uint8_t r, uint8_t v){
  Wire.beginTransmission(a);
  Wire.write(r); Wire.write(v);
  return Wire.endTransmission()==0;
}
bool readBytes(uint8_t a,uint8_t r,uint8_t* b,size_t n){
  Wire.beginTransmission(a); Wire.write(r);
  if(Wire.endTransmission(false)!=0) return false;
  if(Wire.requestFrom((int)a,(int)n)!=n) return false;
  for(size_t i=0;i<n;i++) b[i]=Wire.read();
  return true;
}
bool findIMU(){
  for(uint8_t a:{0x6A,0x6B}){
    uint8_t w=0;
    if(readBytes(a,REG_WHOAMI,&w,1) && w!=0){
      imuAddr=a; return true;
    }
  }
  return false;
}
bool initIMU(){
  if(!findIMU()) return false;
  writeReg(imuAddr,REG_CTRL1,0x40);
  writeReg(imuAddr,REG_CTRL2,0x16);
  writeReg(imuAddr,REG_CTRL3,0x26);
  writeReg(imuAddr,REG_CTRL5,0x11);
  writeReg(imuAddr,REG_CTRL7,0x03);
  return true;
}
bool readAccel(int16_t &ax,int16_t &ay,int16_t &az){
  uint8_t b[6];
  if(!readBytes(imuAddr,REG_AX_L,b,6)) return false;
  ax=(b[1]<<8)|b[0];
  ay=(b[3]<<8)|b[2];
  az=(b[5]<<8)|b[4];
  return true;
}

// ===================== High-pass NUDGE =====================
// EMA baseline removes gravity & slow tilt
float emaX=0, emaY=0;
float emaAlpha = 0.02f;   // smaller = more tilt rejection

float hpX=0, hpY=0;
float nudgeMag=0;

// Min/max capture (high-pass values)
float minX=1e9, maxX=-1e9;
float minY=1e9, maxY=-1e9;
float minM=1e9, maxM=-1e9;

void resetMinMax(){
  minX=1e9; maxX=-1e9;
  minY=1e9; maxY=-1e9;
  minM=1e9; maxM=-1e9;
}

// thresholds you found
const float TH_YELLOW = 200.0f;
const float TH_RED    = 300.0f;

CRGB nudgeColor(float m){
  if (m >= TH_RED)    return CRGB::Red;
  if (m >= TH_YELLOW) return CRGB::Yellow;
  return CRGB::Green;
}

void updateIMU(){
  if(!imuOK) return;

  int16_t ax, ay, az;
  if(!readAccel(ax,ay,az)) return;

  // High-pass (remove baseline)
  emaX += emaAlpha * ((float)ax - emaX);
  emaY += emaAlpha * ((float)ay - emaY);
  hpX = (float)ax - emaX;
  hpY = (float)ay - emaY;

  // magnitude (overall nudge strength)
  nudgeMag = sqrtf(hpX*hpX + hpY*hpY);

  // min/max capture
  minX = min(minX, hpX); maxX = max(maxX, hpX);
  minY = min(minY, hpY); maxY = max(maxY, hpY);
  minM = min(minM, nudgeMag); maxM = max(maxM, nudgeMag);
}

// ===================== Matrix Snake =====================
uint8_t snakeHead = 0;
uint32_t lastSnakeMs = 0;

void snakeStep(uint32_t now){
  // speed: faster when you nudge (fun feedback)
  // base ~70ms, down to ~25ms on big hits
  uint16_t stepMs = (uint16_t)constrain( map((int)min(nudgeMag, 800.0f), 0, 800, 70, 25), 20, 120 );

  if (now - lastSnakeMs < stepMs) return;
  lastSnakeMs = now;

  // trail fade
  fadeToBlackBy(leds, NUM_LEDS, 55);

  // advance head
  snakeHead = (snakeHead + 1) % NUM_LEDS;

  // map head index to logical x,y (row-major logical)
  uint8_t x = snakeHead % W;
  uint8_t y = snakeHead / W;

  drawPixel(x, y, nudgeColor(nudgeMag));
  FastLED.show();
}

// ===================== WEB =====================
WebServer server(80);
const char* AP_SSID="NUDGE-SCOPE";
const char* AP_PASS="pinhalla";

const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Nudge Scope</title>
<style>
body{font-family:system-ui;background:#0b0f14;color:#e8eef6;margin:0}
.wrap{max-width:1000px;margin:auto;padding:14px}
.card{background:#121a24;border:1px solid #243245;border-radius:16px;padding:12px;margin-bottom:12px}
.row{display:grid;grid-template-columns:160px 1fr 70px;gap:8px;align-items:center;margin:6px 0}
canvas{width:100%;height:auto;background:#0b0f14;border-radius:12px;border:1px solid #243245}
button,input{background:#1f2b3a;color:#e8eef6;border:1px solid #243245;border-radius:10px;padding:8px}
.small{opacity:.8;font-size:12px}
</style></head><body>
<div class=wrap>
<div class=card>
<h2 id=ttl>NUDGE SCOPE</h2>
<div class=small>Green &lt; 200, Yellow 200–300, Red &gt; 300 (matches matrix snake)</div>

<div>Rate <b id=hz>0</b> Hz</div>

<div class=row>
<label>Label</label><input id=lbl value="NUDGE SCOPE"><div></div>
</div>

<div class=row>
<label>Update ms</label><input id=ms type=range min=10 max=200 value=25><div id=msv>25</div>
</div>

<div class=row>
<label>Y range</label><input id=yr type=range min=50 max=5000 step=10 value=500><div id=yrv>500</div>
</div>

<button onclick="resetMM()">Reset Min/Max</button>

<div class=small style="margin-top:8px">
X min/max: <b id=xmin>—</b> / <b id=xmax>—</b><br>
Y min/max: <b id=ymin>—</b> / <b id=ymax>—</b><br>
Mag min/max: <b id=mmin>—</b> / <b id=mmax>—</b>
</div>
</div>

<div class=card><h3>X Nudge (left/right)</h3>
<canvas id=gx width=900 height=220></canvas></div>
<div class=card><h3>Y Nudge (forward/back)</h3>
<canvas id=gy width=900 height=220></canvas></div>
</div>

<script>
const gx=document.getElementById('gx').getContext('2d');
const gy=document.getElementById('gy').getContext('2d');
let bx=[],by=[],frames=0,lastT=performance.now();

function draw(ctx,buf,range){
  const w=ctx.canvas.width,h=ctx.canvas.height;
  ctx.clearRect(0,0,w,h);

  // center line
  ctx.strokeStyle='#243245';
  ctx.beginPath();ctx.moveTo(0,h/2);ctx.lineTo(w,h/2);ctx.stroke();

  // threshold lines at +/-200 and +/-300 (scaled to your range)
  function tline(v, col){
    const y=h/2-(v/range)*(h*0.45);
    ctx.strokeStyle=col;
    ctx.setLineDash([6,4]);
    ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke();
    ctx.setLineDash([]);
  }
  tline( 200,'#ffaa00'); tline(-200,'#ffaa00');
  tline( 300,'#ff6666'); tline(-300,'#ff6666');

  // trace
  ctx.strokeStyle='#7cffaa';ctx.lineWidth=2;ctx.beginPath();
  for(let i=0;i<buf.length;i++){
    const x=i/(buf.length-1)*w;
    const y=h/2-(buf[i]/range)*(h*0.45);
    if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);
  }
  ctx.stroke();
}

async function resetMM(){ await fetch('/reset',{cache:'no-store'}); }

async function tick(){
  const ms=+msEl.value, range=+yr.value;

  try{
    const r=await fetch('/imu',{cache:'no-store'});
    const j=await r.json();

    bx.push(j.x); by.push(j.y);
    if(bx.length>300){bx.shift();by.shift();}

    draw(gx,bx,range);
    draw(gy,by,range);

    xmin.textContent=j.xmin?.toFixed(0)??'—';
    xmax.textContent=j.xmax?.toFixed(0)??'—';
    ymin.textContent=j.ymin?.toFixed(0)??'—';
    ymax.textContent=j.ymax?.toFixed(0)??'—';
    mmin.textContent=j.mmin?.toFixed(0)??'—';
    mmax.textContent=j.mmax?.toFixed(0)??'—';

    frames++;
    const now=performance.now();
    if(now-lastT>1000){
      hz.textContent=Math.round(frames*1000/(now-lastT));
      frames=0; lastT=now;
    }
  }catch(e){}
  setTimeout(tick,ms);
}

const msEl=document.getElementById('ms');
const yr=document.getElementById('yr');
msEl.oninput=()=>msv.textContent=msEl.value;
yr.oninput=()=>yrv.textContent=yr.value;
lbl.oninput=()=>ttl.textContent=lbl.value;
msv.textContent=msEl.value; yrv.textContent=yr.value;

tick();
</script></body></html>
)HTML";

void handleRoot(){ server.send(200,"text/html",FPSTR(PAGE)); }

void handleReset(){ resetMinMax(); server.send(200,"text/plain","ok"); }

void handleIMU(){
  // always return last values (even if IMU glitches)
  static float lastX=0,lastY=0,lastM=0;
  static uint16_t failCount=0;

  if (imuOK) {
    int16_t ax, ay, az;
    if (readAccel(ax,ay,az)) {
      failCount = 0;

      emaX += emaAlpha * ((float)ax - emaX);
      emaY += emaAlpha * ((float)ay - emaY);

      hpX = (float)ax - emaX;
      hpY = (float)ay - emaY;
      nudgeMag = sqrtf(hpX*hpX + hpY*hpY);

      lastX = hpX; lastY = hpY; lastM = nudgeMag;

      minX=min(minX,hpX); maxX=max(maxX,hpX);
      minY=min(minY,hpY); maxY=max(maxY,hpY);
      minM=min(minM,nudgeMag); maxM=max(maxM,nudgeMag);
    } else {
      failCount++;
    }
  } else {
    failCount++;
  }

  // auto re-init if repeated fails
  if (failCount >= 25) {
    imuOK = initIMU();
    failCount = 0;
  }

  bool haveMM = (minX < 1e8f);

  String j="{";
  j+="\"x\":"+String(lastX,2)+",\"y\":"+String(lastY,2)+",\"m\":"+String(lastM,2)+",";
  if(haveMM){
    j+="\"xmin\":"+String(minX,2)+",\"xmax\":"+String(maxX,2)+",";
    j+="\"ymin\":"+String(minY,2)+",\"ymax\":"+String(maxY,2)+",";
    j+="\"mmin\":"+String(minM,2)+",\"mmax\":"+String(maxM,2);
  }else{
    j+="\"xmin\":null,\"xmax\":null,\"ymin\":null,\"ymax\":null,\"mmin\":null,\"mmax\":null";
  }
  j+="}";
  server.send(200,"application/json",j);
}

// ===================== SETUP/LOOP =====================
void setup(){
  Serial.begin(115200);
  delay(200);

  // Matrix
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  applyBrightnessCap();
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  // I2C + IMU bring-up
  Wire.begin(I2C_SDA,I2C_SCL);
  delay(50);
  Wire.setClock(400000);
  delay(10);
  imuOK = initIMU();
  delay(10);

  Serial.print("IMU addr: 0x"); Serial.println(imuAddr, HEX);
  Serial.print("IMU OK: "); Serial.println(imuOK ? "true" : "false");

  resetMinMax();

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Web
  server.on("/", handleRoot);
  server.on("/imu", handleIMU);
  server.on("/reset", handleReset);

  // captive portal helpers
  server.on("/generate_204", handleRoot);
  server.on("/hotspot-detect.html", handleRoot);
  server.on("/connecttest.txt", [](){ server.send(200, "text/plain", "Microsoft Connect Test"); });
  server.on("/ncsi.txt", [](){ server.send(200, "text/plain", "Microsoft NCSI"); });

  server.begin();
}

void loop(){
  server.handleClient();

  // update IMU continuously for matrix feedback
  updateIMU();

  // snake color = current magnitude zone
  snakeStep(millis());
}
