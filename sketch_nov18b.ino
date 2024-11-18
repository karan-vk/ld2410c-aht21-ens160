#include <MyLD2410.h>
#include <SparkFun_ENS160.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>

// Pin and Serial Definitions
#define RX_PIN 22
#define TX_PIN 23
#define SERIAL_BAUD_RATE 115200
#define LD2410_BAUD_RATE 256000
#define SDA_PIN 6
#define SCL_PIN 7
#define ENS160_ADDRESS 0x53

// Create sensor instance using Serial1
#define sensorSerial Serial1
MyLD2410 sensor(sensorSerial);
SparkFun_ENS160 myENS;
Adafruit_AHTX0 aht;

// Structure to hold sensor parameters
struct SensorParameters
{
  bool fineResolution;
  byte noOneWindow;
  MyLD2410::ValuesArray movingThresholds;
  MyLD2410::ValuesArray stationaryThresholds;
  byte maxMovingGate;
  byte maxStationaryGate;
};

// Sensor Data Structure
struct EnvironmentalData
{
  int tempC;
  int tempF;
  int humidity;
  int airQuality;
  int tvoc;
  int eco2;
};

// Global variables
SensorParameters originalParams;
unsigned long nextPrint = 0;
const unsigned long printEvery = 1000;

// Function Prototypes
bool initializeENS160();
bool initializeAHT();
void readAndPrintENSData(EnvironmentalData &data);
void readAndPrintAHTData(EnvironmentalData &data);
void printSensorData(const EnvironmentalData &data);

// Helper function to print values
void printValue(const byte &val)
{
  Serial.print(' ');
  Serial.print(val);
}

// Function to print all sensor parameters
void printParameters(bool includeMetadata = false)
{
  if (includeMetadata)
  {
    Serial.print("Firmware: ");
    Serial.println(sensor.getFirmware());
    Serial.print("Protocol version: ");
    Serial.println(sensor.getVersion());
    Serial.print("Bluetooth MAC address: ");
    Serial.println(sensor.getMACstr());
  }

  const MyLD2410::ValuesArray &mThr = sensor.getMovingThresholds();
  const MyLD2410::ValuesArray &sThr = sensor.getStationaryThresholds();

  Serial.print("Resolution (gate-width): ");
  Serial.print(sensor.getResolution());
  Serial.print("cm\nMax range: ");
  Serial.print(sensor.getRange_cm());

  Serial.print("cm\nMoving thresholds    [0,");
  Serial.print(mThr.N);
  Serial.print("]:");
  mThr.forEach(printValue);

  Serial.print("\nStationary thresholds[0,");
  Serial.print(sThr.N);
  Serial.print("]:");
  sThr.forEach(printValue);

  Serial.print("\nNo-one window: ");
  Serial.print(sensor.getNoOneWindow());
  Serial.println('s');
}

// Function to store current parameters
void storeCurrentParameters()
{
  originalParams.fineResolution = (sensor.getResolution() == 20);
  originalParams.noOneWindow = sensor.getNoOneWindow();
  originalParams.movingThresholds = sensor.getMovingThresholds();
  originalParams.stationaryThresholds = sensor.getStationaryThresholds();
  originalParams.maxMovingGate = sensor.getMovingThresholds().N;
  originalParams.maxStationaryGate = sensor.getStationaryThresholds().N;
}

// Function to restore original parameters
bool restoreParameters()
{
  Serial.println("\nRestoring original settings...");
  delay(1000);

  bool success = true;

  // Restore resolution
  if (!sensor.setResolution(originalParams.fineResolution))
  {
    Serial.println("Failed to restore resolution");
    success = false;
  }

  // Restore gates and thresholds
  if (!sensor.setGateParameters(
          originalParams.movingThresholds,
          originalParams.stationaryThresholds,
          originalParams.noOneWindow))
  {
    Serial.println("Failed to restore gate parameters");
    success = false;
  }

  if (success)
  {
    Serial.println("Parameters restored successfully!");
    printParameters();
  }
  else
  {
    Serial.println("Some parameters failed to restore.");
    Serial.println("Consider using factory_reset sketch to restore defaults.");
  }

  return success;
}

// Function to apply new parameters
bool applyParameters(const SensorParameters &params)
{
  Serial.println("\nApplying new parameters...");
  bool success = true;

  // Set no-one window
  if (!sensor.setNoOneWindow(params.noOneWindow))
  {
    Serial.println("Failed to set no-one window");
    success = false;
  }

  // Set resolution
  if (!sensor.setResolution(params.fineResolution))
  {
    Serial.println("Failed to set resolution");
    success = false;
  }

  // Set maximum gates
  if (!sensor.setMaxMovingGate(params.maxMovingGate))
  {
    Serial.println("Failed to set max moving gate");
    success = false;
  }

  if (!sensor.setMaxStationaryGate(params.maxStationaryGate))
  {
    Serial.println("Failed to set max stationary gate");
    success = false;
  }

  // Set gate parameters
  if (!sensor.setGateParameters(
          params.movingThresholds,
          params.stationaryThresholds,
          params.noOneWindow))
  {
    Serial.println("Failed to set gate parameters");
    success = false;
  }

  if (success)
  {
    Serial.println("Parameters applied successfully!");
    printParameters();
  }
  else
  {
    Serial.println("Some parameters failed to apply.");
  }

  return success;
}

// Function to initialize sensor
bool initializeSensor()
{
  Serial.begin(SERIAL_BAUD_RATE);
  sensorSerial.begin(LD2410_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(2000);

  if (!sensor.begin())
  {
    Serial.println("Failed to communicate with the sensor.");
    return false;
  }

  Serial.println("Sensor initialized successfully!");
  return true;
}

// Function to print sensor data
void printSensorData()
{
  Serial.print(sensor.statusString());

  if (sensor.presenceDetected())
  {
    Serial.print(", distance: ");
    Serial.print(sensor.detectedDistance());
    Serial.print("cm");
  }
  Serial.println();

  if (sensor.movingTargetDetected())
  {
    Serial.print(" MOVING    = ");
    Serial.print(sensor.movingTargetSignal());
    Serial.print("@");
    Serial.print(sensor.movingTargetDistance());
    Serial.print("cm ");

    if (sensor.inEnhancedMode())
    {
      Serial.print("\n signals->[");
      sensor.getMovingSignals().forEach(printValue);
      Serial.print(" ] thresholds:[");
      sensor.getMovingThresholds().forEach(printValue);
      Serial.print(" ]");
    }
    Serial.println();
  }

  if (sensor.stationaryTargetDetected())
  {
    Serial.print(" STATIONARY= ");
    Serial.print(sensor.stationaryTargetSignal());
    Serial.print("@");
    Serial.print(sensor.stationaryTargetDistance());
    Serial.print("cm ");

    if (sensor.inEnhancedMode())
    {
      Serial.print("\n signals->[");
      sensor.getStationarySignals().forEach(printValue);
      Serial.print(" ] thresholds:[");
      sensor.getStationaryThresholds().forEach(printValue);
      Serial.print(" ]");
    }
    Serial.println();
  }

  byte lightLevel = sensor.getLightLevel();
  if (lightLevel)
  {
    Serial.print("Light level: ");
    Serial.println(lightLevel);
  }
  Serial.println();
}
// Function to monitor detection quality
void monitorDetectionQuality()
{
  unsigned long startTime = millis();
  const unsigned long monitorDuration = 10000; // 10 seconds monitoring
  int detectionCount = 0;
  int falsePositives = 0;

  Serial.println("\nMonitoring detection quality for 10 seconds...");

  while (millis() - startTime < monitorDuration)
  {
    if (sensor.check() == MyLD2410::Response::DATA)
    {
      if (sensor.presenceDetected())
      {
        detectionCount++;

        // Log detection details
        Serial.print("Detection at distance: ");
        Serial.print(sensor.detectedDistance());
        Serial.println("cm");

        if (sensor.detectedDistance() > 300)
        {
          // Log potential false positive
          falsePositives++;
          Serial.println("Warning: Detection beyond room dimensions!");
        }
      }
      delay(100);
    }
  }

  Serial.println("\nDetection Quality Report:");
  Serial.print("Total detections: ");
  Serial.println(detectionCount);
  Serial.print("Potential false positives: ");
  Serial.println(falsePositives);
}

void setup()
{
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!initializeSensor())
  {
    while (true)
    {
    } // Halt if initialization fails
  }

  Serial.println("Initializing sensors...");

  // Initialize ENS160
  if (!initializeENS160())
  {
    Serial.println("ERROR: ENS160 initialization failed");
    while (1)
      delay(10); // Halt if sensor init fails
  }

  // Initialize AHT
  if (!initializeAHT())
  {
    Serial.println("ERROR: AHT initialization failed");
    while (1)
      delay(10); // Halt if sensor init fails
  }

  // Force fine resolution first (20cm gates)
  if (sensor.getResolution() != 20)
  {
    if (!sensor.setResolution(true))
    {
      Serial.println("Failed to set fine resolution");
      while (true)
      {
      }
    }
    delay(100);
  }

  // Store original parameters
  storeCurrentParameters();
  printParameters(true);

  // Configure for 250cm x 300cm room
  SensorParameters roomParams;
  roomParams.fineResolution = true; // 20cm resolution
  roomParams.noOneWindow = 2;       // 2 second no-one window (adjustable based on needs)

  // Set maximum detection range to cover the room diagonal
  // diagonal = √(250² + 300²) ≈ 390cm
  // Using 15 gates to cover max dimension (300cm)
  roomParams.maxMovingGate = 15;     // Cover 300cm for moving targets
  roomParams.maxStationaryGate = 15; // Cover 300cm for stationary targets

  // Get current thresholds as base
  roomParams.movingThresholds = sensor.getMovingThresholds();
  roomParams.stationaryThresholds = sensor.getStationaryThresholds();

  // Configure gate sensitivities
  // Lower values = more sensitive, Higher values = less sensitive
  // Adjust these thresholds based on your needs
  for (int i = 0; i <= 15; i++)
  {
    if (i < roomParams.movingThresholds.N)
    {
      // Configure moving target sensitivity
      if (i <= 5)
      {
        // Close range (0-100cm): higher threshold to avoid false triggers
        roomParams.movingThresholds.values[i] = 40;
      }
      else if (i <= 10)
      {
        // Mid range (100-200cm): medium threshold
        roomParams.movingThresholds.values[i] = 35;
      }
      else
      {
        // Far range (200-300cm): lower threshold to better detect
        roomParams.movingThresholds.values[i] = 30;
      }
    }

    if (i < roomParams.stationaryThresholds.N)
    {
      // Configure stationary target sensitivity
      if (i <= 5)
      {
        // Close range: higher threshold
        roomParams.stationaryThresholds.values[i] = 50;
      }
      else if (i <= 10)
      {
        // Mid range: medium threshold
        roomParams.stationaryThresholds.values[i] = 45;
      }
      else
      {
        // Far range: lower threshold
        roomParams.stationaryThresholds.values[i] = 40;
      }
    }
  }

  // Apply the new configuration
  if (!applyParameters(roomParams))
  {
    Serial.println("Failed to apply room parameters!");
    return;
  }

  Serial.println("\nRoom configuration complete!");
  Serial.println("Room dimensions: 250cm x 300cm");
  Serial.println("Coverage using 20cm gates:");
  Serial.println("- Width: 13 gates (260cm)");
  Serial.println("- Length: 15 gates (300cm)");

  // Add monitoring function to check detection reliability
  monitorDetectionQuality();
}

void loop()
{
  EnvironmentalData sensorData = {0};
  if ((sensor.check() == MyLD2410::Response::DATA) && (millis() > nextPrint))
  {
    nextPrint = millis() + printEvery;

    // Enhanced data printing for room monitoring
    Serial.println("\n--- Room Monitoring Data ---");
    if (sensor.presenceDetected())
    {
      Serial.print("Target detected at: ");
      Serial.print(sensor.detectedDistance());
      Serial.println("cm");

      // Add position context
      float detectedDistance = sensor.detectedDistance();
      if (detectedDistance <= 100)
      {
        Serial.println("Position: Near entrance");
      }
      else if (detectedDistance <= 200)
      {
        Serial.println("Position: Room middle");
      }
      else
      {
        Serial.println("Position: Far end");
      }
    }

    printSensorData();
  }
  readAndPrintENSData(sensorData);
  readAndPrintAHTData(sensorData);
  delay(1000);
}

bool initializeENS160()
{
  Serial.println("Initializing ENS160 sensor...");

  if (!myENS.begin())
  {
    return false;
  }

  // Reset and configure sensor
  if (!myENS.setOperatingMode(SFE_ENS160_RESET))
  {
    return false;
  }

  delay(100); // Allow time for reset

  // Set to standard operation mode
  if (!myENS.setOperatingMode(SFE_ENS160_STANDARD))
  {
    return false;
  }

  // Check sensor status
  int ensStatus = myENS.getFlags();
  Serial.print("Gas Sensor Status Flag: ");
  Serial.println(ensStatus);

  // Status interpretation
  const char *statusMessages[] = {
      "Operating OK",
      "Warm-up phase",
      "Initial Start-up",
      "No Valid Output"};

  if (ensStatus >= 0 && ensStatus <= 3)
  {
    Serial.println(statusMessages[ensStatus]);
  }

  return true;
}

bool initializeAHT()
{
  Serial.println("Initializing AHT sensor...");
  if (!aht.begin())
  {
    return false;
  }
  Serial.println("AHT sensor initialized successfully");
  return true;
}

void readAndPrintENSData(EnvironmentalData &data)
{
  if (myENS.checkDataStatus())
  {
    data.airQuality = myENS.getAQI();
    data.tvoc = myENS.getTVOC();
    data.eco2 = myENS.getECO2();

    Serial.println("\n=== Air Quality Measurements ===");
    Serial.print("Air Quality Index (1-5) : ");
    Serial.println(data.airQuality);
    Serial.print("TVOC: ");
    Serial.print(data.tvoc);
    Serial.println(" ppb");
    Serial.print("CO2: ");
    Serial.print(data.eco2);
    Serial.println(" ppm");
  }
  else
  {
    Serial.println("WARNING: ENS160 data not ready");
  }
}

void readAndPrintAHTData(EnvironmentalData &data)
{
  sensors_event_t humidity_event, temp_event;
  aht.getEvent(&humidity_event, &temp_event);

  data.tempC = temp_event.temperature;
  data.tempF = (temp_event.temperature * 1.8) + 32;
  data.humidity = humidity_event.relative_humidity;

  Serial.println("\n=== Temperature and Humidity ===");
  Serial.print("Temperature: ");
  Serial.print(data.tempC);
  Serial.println("°C");
  Serial.print("Temperature: ");
  Serial.print(data.tempF);
  Serial.println("°F");
  Serial.print("Humidity: ");
  Serial.print(data.humidity);
  Serial.println("% rH");
}