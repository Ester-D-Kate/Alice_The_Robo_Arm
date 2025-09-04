import asyncio
import json
import os
from datetime import datetime
from io import BytesIO
from typing import Dict, Optional

import uvicorn
from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from groq import Groq
from paho.mqtt.client import Client as MQTTClient

from dotenv import load_dotenv
load_dotenv()

app = FastAPI(title="Voice Robot Controller", version="2.0.0")

# CORS middleware for web interface
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize Groq client
groq_client = Groq(api_key=os.getenv("GROQ_API_KEY"))

# MQTT Configuration
MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883
CAR_TOPIC = "alice/zero/car/command"
ARM_TOPIC = "alice/zero/arm/command"
ARM_POSITION_TOPIC = "alice/zero/arm/position"

# Global MQTT client
mqtt_client = None
command_count = 0

def connect_mqtt():
    """Connect to MQTT broker"""
    global mqtt_client
    try:
        # Fixed: Remove callback_api_version parameter for newer paho-mqtt versions
        mqtt_client = MQTTClient(f"VoiceController_{datetime.now().timestamp()}")
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
        print(f"✅ Connected to MQTT broker: {MQTT_BROKER}")
        return True
    except Exception as e:
        print(f"❌ MQTT connection failed: {e}")
        return False

def send_command(topic: str, command: str, speed: Optional[int] = None, positions: Optional[Dict] = None):
    """Send command via MQTT"""
    global command_count, mqtt_client
    
    if not mqtt_client:
        return False
    
    command_count += 1
    
    if topic == ARM_POSITION_TOPIC and positions:
        # Position command for arm
        payload = positions.copy()
        payload["timestamp"] = int(datetime.now().timestamp() * 1000)
        payload["id"] = command_count
    else:
        # Regular command
        payload = {
            "cmd": command,
            "timestamp": int(datetime.now().timestamp() * 1000),
            "id": command_count
        }
        
        if speed is not None and topic == CAR_TOPIC:
            payload["speed"] = speed
    
    try:
        mqtt_client.publish(topic, json.dumps(payload))
        print(f"📤 Sent: {command} to {topic} {f'(speed: {speed}%)' if speed else ''}")
        return True
    except Exception as e:
        print(f"❌ Failed to send command: {e}")
        return False

def text_to_robot_commands(text: str) -> Dict[str, str]:
    """Convert natural language to robot commands using Groq"""
    
    system_prompt = """You are a robot command translator. Convert natural language to specific robot commands.

ROBOT SYSTEM:
- CAR: Moves the entire robot base (has speed control 0-100%)
- ARM: 4 joints - Claw, Claw Neck, Arm Joint 1, Arm Joint 2 (no speed, moves incrementally)

CAR COMMANDS:
- F (forward), B (backward), L (left), R (right), S (stop)
- Speed: 0-100% (slow=25%, normal=50%, fast=80%, max=100%)

ARM COMMANDS (Manual/Incremental):
- U (arm up - both joints move), D (arm down - both joints move)
- C (close claw), O (open claw)
- 1 (claw neck left), 4 (claw neck right) 
- 2 (claw neck up), 3 (claw neck down)
- S (stop all arm movement)

ARM POSITION COMMANDS (Absolute):
- "move claw to 45 degrees" → position command
- "set arm joint 1 to 90" → position command
- A1=Claw (0-180°), A2=Claw Neck (0-180°), A3=Arm Joint 1 (0-180°), A4=Arm Joint 2 (0-180°)

SPEED DETECTION (CAR ONLY):
- "slowly" = 25%, "fast" = 80%, "very fast" = 100%
- Numbers: "speed 70" = 70%
- Default = 50%

DURATION:
- "for X seconds" = X duration
- "briefly" = 1 second, "long" = 5 seconds
- Default = 2 seconds

RESPONSE FORMAT (JSON only):
{
    "command_type": "manual" OR "position" OR "car",
    "car_command": "F" OR null,
    "arm_command": "U" OR null,
    "positions": {"A1": 90, "A2": 45} OR null,
    "speed": 50,
    "duration": 2,
    "interpretation": "description"
}

RULES:
- Car commands: Use "car_command" and "speed"
- Arm manual: Use "arm_command" (no speed)
- Arm position: Use "positions" with joint angles
- Speed ONLY affects car movement
- If angle mentioned, use position command
- If direction mentioned (up/down/open/close), use manual command

EXAMPLES:
"move forward slowly" → {"command_type": "car", "car_command": "F", "arm_command": null, "positions": null, "speed": 25, "duration": 2, "interpretation": "moving car forward slowly"}

"arm up" → {"command_type": "manual", "car_command": null, "arm_command": "U", "positions": null, "speed": 50, "duration": 2, "interpretation": "moving arm up incrementally"}

"move claw to 45 degrees" → {"command_type": "position", "car_command": null, "arm_command": null, "positions": {"A1": 45}, "speed": 50, "duration": 3, "interpretation": "moving claw to 45 degrees"}

"close gripper" → {"command_type": "manual", "car_command": null, "arm_command": "C", "positions": null, "speed": 50, "duration": 1, "interpretation": "closing claw"}

"turn left fast and close claw" → {"command_type": "manual", "car_command": "L", "arm_command": "C", "positions": null, "speed": 80, "duration": 2, "interpretation": "turning left fast while closing claw"}

"set arm joint 1 to 90 degrees" → {"command_type": "position", "car_command": null, "arm_command": null, "positions": {"A3": 90}, "speed": 50, "duration": 3, "interpretation": "setting arm joint 1 to 90 degrees"}"""

    try:
        response = groq_client.chat.completions.create(
            model="llama-3.1-8b-instant",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": text}
            ],
            temperature=0.2,
            max_tokens=300
        )
        
        result_text = response.choices[0].message.content.strip()
        
        # Extract JSON from response
        if result_text.startswith("```json"):
            result_text = result_text[7:-3]
        elif result_text.startswith("```"):
            result_text = result_text[3:-3]
            
        parsed_result = json.loads(result_text)
        
        # Ensure all required fields exist
        required_fields = ["command_type", "car_command", "arm_command", "positions", "speed", "duration", "interpretation"]
        for field in required_fields:
            if field not in parsed_result:
                if field == "speed":
                    parsed_result[field] = 50
                elif field == "duration":
                    parsed_result[field] = 20
                elif field == "command_type":
                    parsed_result[field] = "manual"
                else:
                    parsed_result[field] = None
        
        # Validate ranges
        if parsed_result["speed"]:
            parsed_result["speed"] = max(0, min(100, int(parsed_result["speed"])))
        if parsed_result["duration"]:
            parsed_result["duration"] = max(0.5, min(10, float(parsed_result["duration"])))
        
        # Validate positions
        if parsed_result["positions"]:
            for joint, angle in parsed_result["positions"].items():
                if joint in ["A1", "A2", "A3", "A4"]:
                    parsed_result["positions"][joint] = max(0, min(180, int(angle)))
        
        return parsed_result
        
    except Exception as e:
        print(f"❌ Command translation failed: {e}")
        return {
            "command_type": "manual",
            "car_command": "S",
            "arm_command": "S", 
            "positions": None,
            "speed": 50,
            "duration": 1,
            "interpretation": f"Error: {str(e)}"
        }

@app.on_event("startup")
async def startup_event():
    """Initialize MQTT connection on startup"""
    connect_mqtt()

@app.post("/voice-command")
async def process_voice_command(audio_file: UploadFile = File(...)):
    """Process voice command and return parsed commands for web interface execution"""
    
    if not audio_file.content_type.startswith('audio/'):
        raise HTTPException(status_code=400, detail="Please upload an audio file")
    
    try:
        # Read audio file
        audio_data = await audio_file.read()
        
        # Transcribe with Groq Whisper
        print("🎤 Transcribing audio...")
        transcription = groq_client.audio.transcriptions.create(
            file=(audio_file.filename, BytesIO(audio_data), audio_file.content_type),
            model="whisper-large-v3",
            language="en"
        )
        
        text = transcription.text.strip()
        print(f"📝 Transcribed: '{text}'")
        
        if not text:
            raise HTTPException(status_code=400, detail="No speech detected")
        
        # Convert to robot commands
        print("🤖 Converting to robot commands...")
        commands = text_to_robot_commands(text)
        
        print(f"🎯 Command Type: {commands['command_type']}")
        print(f"🎯 Car: {commands['car_command']}, Arm: {commands['arm_command']}, Positions: {commands['positions']}")
        print(f"🎯 Speed: {commands['speed']}%, Duration: {commands['duration']}s")
        
        # Return commands for web interface to execute
        return {
            "success": True,
            "transcription": text,
            "commands": commands,
            "timestamp": datetime.now().isoformat()
        }
        
    except Exception as e:
        print(f"❌ Voice command processing failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/text-command")
async def process_text_command(text: str):
    """Process text command directly (for testing)"""
    try:
        commands = text_to_robot_commands(text)
        
        # Execute commands via MQTT
        success = True
        
        # Car command
        if commands.get("car_command") and commands["car_command"] != "S":
            success &= send_command(CAR_TOPIC, commands["car_command"], commands.get("speed"))
        
        # Arm commands
        if commands.get("command_type") == "position" and commands.get("positions"):
            # Position command
            success &= send_command(ARM_POSITION_TOPIC, "", positions=commands["positions"])
        elif commands.get("arm_command") and commands["arm_command"] != "S":
            # Manual command
            success &= send_command(ARM_TOPIC, commands["arm_command"])
        
        # Auto-stop after duration
        if commands.get("duration", 0) > 0:
            asyncio.create_task(send_stop_after_delay(commands["duration"]))
        
        return {
            "success": success,
            "commands": commands,
            "timestamp": datetime.now().isoformat()
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

async def send_stop_after_delay(delay_seconds: float):
    """Send stop commands after specified delay"""
    await asyncio.sleep(delay_seconds)
    send_command(CAR_TOPIC, "S")
    send_command(ARM_TOPIC, "S")
    print(f"⏹️ Auto-stop after {delay_seconds}s")

@app.post("/emergency-stop")
async def emergency_stop():
    """Emergency stop all robot movement"""
    car_success = send_command(CAR_TOPIC, "S")
    arm_success = send_command(ARM_TOPIC, "S")
    
    return {
        "success": car_success and arm_success,
        "message": "Emergency stop executed",
        "timestamp": datetime.now().isoformat()
    }

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "mqtt_connected": mqtt_client is not None,
        "timestamp": datetime.now().isoformat()
    }

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)