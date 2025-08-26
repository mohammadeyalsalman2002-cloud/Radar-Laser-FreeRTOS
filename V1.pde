import processing.serial.*;

Serial myPort;

float angle = 0;
float distance = 0;
int detected = 0;

float maxDistance = 100; // Maximum distance (cm)
float alertDistance = 50; // Danger distance
float scaleFactor;
float[][] radarMap;

ArrayList<Trail> sweepTrail = new ArrayList<Trail>();

void setup() {
  size(1200, 700); 
  myPort = new Serial(this, "COM13", 9600);
  myPort.bufferUntil('\n');

  radarMap = new float[181][2];
  for (int i = 0; i <= 180; i++) {
    radarMap[i][0] = -1;
    radarMap[i][1] = 0;
  }

  scaleFactor = (height - 150) / (maxDistance / 1.0);
}

void draw() {
  background(0);
  translate(width/2, height - 50);

  // Distance arcs
  stroke(0, 255, 0);
  noFill();
  for (float r = 0; r <= maxDistance; r += 5) {
    arc(0, 0, r*scaleFactor*2, r*scaleFactor*2, PI, TWO_PI);
    fill(0, 255, 0);
    textAlign(CENTER, CENTER);
    text((int)r + "cm", 0, -r*scaleFactor);
    noFill();
  }

  // Angle lines
  for (int a = 0; a <= 180; a += 10) {
    float x = cos(radians(a)) * maxDistance * scaleFactor;
    float y = -sin(radians(a)) * maxDistance * scaleFactor;
    line(0, 0, x, y);
    textAlign(CENTER, CENTER);
    text(a + "°", x*1.05, y*1.05);
  }

  // Trail effect
  for (int i = sweepTrail.size() - 1; i >= 0; i--) {
    Trail t = sweepTrail.get(i);
    t.display();
    t.update();
    if (t.alpha <= 0) {
      sweepTrail.remove(i);
    }
  }

  sweepTrail.add(new Trail(angle, maxDistance * scaleFactor));

  // Current sweep line
  stroke(0, 255, 0);
  line(0, 0, cos(radians(angle)) * maxDistance * scaleFactor, 
                 -sin(radians(angle)) * maxDistance * scaleFactor);

  // Draw detected points (مع حجم متغير حسب المسافة)
  noStroke();
  for (int a = 0; a <= 180; a++) {
    if (radarMap[a][0] > 0) {
      float px = cos(radians(a)) * radarMap[a][0] * scaleFactor;
      float py = -sin(radians(a)) * radarMap[a][0] * scaleFactor;

      fill(distanceColor(radarMap[a][0]));

      // حجم النقطة يتغير مع المسافة
      float size = map(radarMap[a][0], 0, maxDistance, 14, 4); 
      ellipse(px, py, size, size);
    }
  }

  drawHUD();
}

void drawHUD() {
  fill(0, 255, 0);
  textAlign(LEFT, TOP);
  textSize(16);
  String status = (distance <= alertDistance && detected == 1) ? "NOT SAFE" : 
                  (distance > alertDistance && distance <= maxDistance && detected == 1) ? "WARNING" : "SAFE";
  text("Angle: " + (int)angle + "°", -width/2 + 20, -height + 80);
  text("Distance: " + (int)distance + " cm", -width/2 + 20, -height + 110);
  text("Status: " + status, -width/2 + 20, -height + 140);
}

// New color logic
color distanceColor(float d) {
  if (d <= alertDistance) {
    return color(255, 0, 0); // Red for danger
  } else if (d > alertDistance && d <= maxDistance) {
    return color(255, 255, 0); // Yellow for warning
  }
  return color(0, 255, 0); // Green (shouldn't normally be used now)
}

void serialEvent(Serial p) {
  String data = p.readStringUntil('\n');
  if (data != null) {
    data = trim(data);
    String[] values = split(data, ',');
    if (values.length == 3) {
      angle = float(values[0]);
      distance = float(values[1]);
      detected = int(values[2]);

      // Store point if object exists within 100 cm
      if (distance >= 0 && distance <= maxDistance) {
        radarMap[(int)angle][0] = distance;
        radarMap[(int)angle][1] = 1;
      } else {
        radarMap[(int)angle][0] = -1;
        radarMap[(int)angle][1] = 0;
      }
    }
  }
}

class Trail {
  float ang;
  float len;
  float alpha = 150;

  Trail(float a, float l) {
    ang = a;
    len = l;
  }

  void update() {
    alpha -= 3;
  }

  void display() {
    stroke(0, 255, 0, alpha);
    line(0, 0, cos(radians(ang)) * len, -sin(radians(ang)) * len);
  }
}
