import asyncio
import json
import os
import random
from datetime import datetime
from io import BytesIO
from typing import Dict, Optional, List

import uvicorn
from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from groq import Groq
from paho.mqtt.client import Client as MQTTClient

from dotenv import load_dotenv
load_dotenv()

app = FastAPI(title="Alice Voice Robot Controller", version="3.0.0")

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
        mqtt_client = MQTTClient(f"AliceController_{datetime.now().timestamp()}")
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
        payload = positions.copy()
        payload["timestamp"] = int(datetime.now().timestamp() * 1000)
        payload["id"] = command_count
    else:
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

def generate_dynamic_sequence(intent: str, style: str = "normal") -> List[Dict]:
    """Generate dynamic sequences based on intent and style using AI"""
    
    sequence_prompt = f"""You are Alice, a creative robotic arm assistant. Generate a sequence of movements for the intent: "{intent}" with style: "{style}".

AVAILABLE COMMANDS:
Car: F(forward), B(backward), L(left), R(right), S(stop)
Arm: U(up), D(down), C(close claw), O(open claw), 1(neck left), 2(neck up), 3(neck down), 4(neck right), S(stop)

STYLES:
- energetic: Fast, dynamic movements with high speed (70-90%)
- gentle: Slow, smooth movements with low speed (20-40%)
- playful: Random, fun combinations with medium speed (50-70%)
- elegant: Graceful, coordinated movements with medium speed (40-60%)
- dramatic: Bold, expressive movements with varying speeds

SEQUENCE RULES:
- Create 4-8 movement steps
- Each step should have car_command, arm_command, speed, and duration
- Duration should be 0.5-3.0 seconds
- Speed should be 20-90% for car movements
- Always end with stop commands
- Make movements meaningful for the intent

RESPONSE FORMAT (JSON only):
[
  {{"car_command": "L", "arm_command": "U", "speed": 60, "duration": 1.5}},
  {{"car_command": "R", "arm_command": "D", "speed": 60, "duration": 1.5}},
  {{"car_command": "S", "arm_command": "S", "speed": 0, "duration": 0.5}}
]

Generate a creative sequence for: {intent} (style: {style})"""

    try:
        response = groq_client.chat.completions.create(
            model="llama-3.1-8b-instant",
            messages=[
                {"role": "system", "content": "You are a creative movement choreographer for Alice robot. Generate JSON sequences only."},
                {"role": "user", "content": sequence_prompt}
            ],
            temperature=0.7,  # Higher creativity
            max_tokens=500
        )
        
        result_text = response.choices[0].message.content.strip()
        
        # Extract JSON from response
        if result_text.startswith("```json"):
            result_text = result_text[7:-3]
        elif result_text.startswith("```"):
            result_text = result_text[3:-3]
        elif result_text.startswith("["):
            pass  # Already JSON
        else:
            # Find JSON array in text
            start = result_text.find("[")
            end = result_text.rfind("]") + 1
            if start != -1 and end > start:
                result_text = result_text[start:end]
        
        sequence = json.loads(result_text)
        
        # Validate and clean sequence
        cleaned_sequence = []
        for step in sequence:
            cleaned_step = {
                "car_command": step.get("car_command", "S"),
                "arm_command": step.get("arm_command", "S"),
                "speed": max(20, min(90, step.get("speed", 50))),
                "duration": max(0.5, min(3.0, step.get("duration", 1.0)))
            }
            cleaned_sequence.append(cleaned_step)
        
        print(f"🎭 Generated {len(cleaned_sequence)} step sequence for: {intent}")
        return cleaned_sequence
        
    except Exception as e:
        print(f"❌ Sequence generation failed: {e}")
        # Fallback simple sequence
        return [
            {"car_command": "L", "arm_command": "U", "speed": 50, "duration": 1},
            {"car_command": "R", "arm_command": "D", "speed": 50, "duration": 1},
            {"car_command": "S", "arm_command": "S", "speed": 0, "duration": 0.5}
        ]

def enhanced_text_to_robot_commands(text: str) -> Dict:
    """Enhanced command translator with dynamic sequence generation"""
    
    system_prompt = """You are Alice's advanced command interpreter. Analyze the user's request and determine the best response type.

COMMAND TYPES:
1. "simple" - Single movement commands
2. "sequence_preset" - Use predefined sequences (dance, wave, spin, patrol)
3. "sequence_dynamic" - Generate new custom sequences
4. "position" - Precise joint positioning

DYNAMIC SEQUENCE TRIGGERS:
- Creative requests: "be creative", "surprise me", "show your personality"
- Emotional expressions: "be happy", "act sad", "show excitement", "be dramatic"
- Activity requests: "exercise", "stretch", "warm up", "cool down"
- Entertainment: "perform", "entertain me", "put on a show", "be funny"
- Character actions: "be a dinosaur", "act like a cat", "pretend to swim"
- Contextual actions: "celebrate victory", "say goodbye", "welcome someone"

STYLE DETECTION:
- "energetic/fast/quick/excited" → energetic
- "gentle/slow/calm/peaceful" → gentle  
- "fun/playful/silly/random" → playful
- "elegant/graceful/smooth" → elegant
- "dramatic/bold/expressive" → dramatic
- Default → normal

PRESET SEQUENCES:
- dance, wave, spin, patrol (use for basic requests)

RESPONSE FORMAT:
{
    "command_type": "simple|sequence_preset|sequence_dynamic|position",
    "car_command": "F" OR null,
    "arm_command": "U" OR null,
    "positions": {"A1": 90} OR null,
    "speed": 50,
    "duration": 2,
    "sequence_name": "dance" OR null,
    "dynamic_intent": "be happy" OR null,
    "style": "energetic" OR null,
    "interpretation": "description"
}

EXAMPLES:
"dance for me" → sequence_preset: "dance"
"be creative and surprise me" → sequence_dynamic, intent: "surprise me", style: "playful"
"show excitement energetically" → sequence_dynamic, intent: "show excitement", style: "energetic"
"act like a happy puppy" → sequence_dynamic, intent: "act like a happy puppy", style: "playful"
"gracefully welcome the guests" → sequence_dynamic, intent: "welcome the guests", style: "elegant"
"move forward" → simple car command"""

    try:
        response = groq_client.chat.completions.create(
            model="llama-3.1-8b-instant",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": text}
            ],
            temperature=0.3,
            max_tokens=300
        )
        
        result_text = response.choices[0].message.content.strip()
        
        # Extract JSON
        if result_text.startswith("```json"):
            result_text = result_text[7:-3]
        elif result_text.startswith("```"):
            result_text = result_text[3:-3]
            
        parsed_result = json.loads(result_text)
        
        # Set defaults
        defaults = {
            "command_type": "simple",
            "car_command": None,
            "arm_command": None,
            "positions": None,
            "speed": 50,
            "duration": 2,
            "sequence_name": None,
            "dynamic_intent": None,
            "style": "normal",
            "interpretation": "Unknown command"
        }
        
        for key, default_value in defaults.items():
            if key not in parsed_result:
                parsed_result[key] = default_value
        
        return parsed_result
        
    except Exception as e:
        print(f"❌ Command interpretation failed: {e}")
        return {
            "command_type": "simple",
            "car_command": "S",
            "arm_command": "S",
            "dynamic_intent": f"Error: {str(e)}",
            "style": "normal",
            "interpretation": "Error occurred"
        }

# Predefined sequences (keeping the originals)
PRESET_SEQUENCES = {
    "dance": [
        {"car_command": "L", "arm_command": "U", "speed": 60, "duration": 1},
        {"car_command": "R", "arm_command": "D", "speed": 60, "duration": 1},
        {"car_command": "L", "arm_command": "U", "speed": 60, "duration": 1},
        {"car_command": "R", "arm_command": "D", "speed": 60, "duration": 1},
        {"car_command": "S", "arm_command": "S", "speed": 0, "duration": 0.5}
    ],
    "wave": [
        {"arm_command": "U", "speed": 50, "duration": 1},
        {"arm_command": "D", "speed": 50, "duration": 1},
        {"arm_command": "U", "speed": 50, "duration": 1},
        {"arm_command": "D", "speed": 50, "duration": 1},
        {"arm_command": "S", "speed": 0, "duration": 0.5}
    ],
    "spin": [
        {"car_command": "L", "speed": 70, "duration": 0.7},
        {"car_command": "L", "speed": 70, "duration": 0.7},
        {"car_command": "L", "speed": 70, "duration": 0.7},
        {"car_command": "L", "speed": 70, "duration": 0.7},
        {"car_command": "S", "speed": 0, "duration": 0.5}
    ],
    "patrol": [
        {"car_command": "F", "speed": 50, "duration": 2},
        {"car_command": "L", "speed": 50, "duration": 1},
        {"car_command": "F", "speed": 50, "duration": 2},
        {"car_command": "R", "speed": 50, "duration": 1},
        {"car_command": "S", "speed": 0, "duration": 0.5}
    ]
}

async def execute_sequence(sequence: List[Dict]) -> bool:
    """Execute any sequence of movements"""
    print(f"🎭 Executing {len(sequence)} step sequence")
    
    for i, step in enumerate(sequence):
        print(f"Step {i+1}: Car={step.get('car_command', 'S')}, Arm={step.get('arm_command', 'S')}, Speed={step.get('speed', 0)}%, Duration={step.get('duration', 0)}s")
        
        # Send car command
        if step.get("car_command") and step["car_command"] != "S":
            send_command(CAR_TOPIC, step["car_command"], step.get("speed", 50))
        
        # Send arm command
        if step.get("arm_command") and step["arm_command"] != "S":
            send_command(ARM_TOPIC, step["arm_command"])
        
        # Wait for step duration
        if step.get("duration", 0) > 0:
            await asyncio.sleep(step["duration"])
        
        # Send stop commands if this is a stop step
        if step.get("car_command") == "S":
            send_command(CAR_TOPIC, "S")
        if step.get("arm_command") == "S":
            send_command(ARM_TOPIC, "S")
    
    print(f"✅ Sequence completed successfully")
    return True

@app.on_event("startup")
async def startup_event():
    """Initialize MQTT connection on startup"""
    connect_mqtt()

@app.post("/voice-command")
async def process_voice_command(audio_file: UploadFile = File(...)):
    """Process voice command with dynamic sequence generation"""
    
    if not audio_file.content_type.startswith('audio/'):
        raise HTTPException(status_code=400, detail="Please upload an audio file")
    
    try:
        # Read and transcribe audio
        audio_data = await audio_file.read()
        
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
        
        # Enhanced command interpretation
        commands = enhanced_text_to_robot_commands(text)
        
        print(f"🎯 Command Type: {commands['command_type']}")
        print(f"🎭 Intent: {commands.get('dynamic_intent', 'N/A')}")
        print(f"🎨 Style: {commands.get('style', 'normal')}")
        
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
    """Process text command with dynamic sequence generation"""
    try:
        commands = enhanced_text_to_robot_commands(text)
        success = True
        
        if commands.get("command_type") == "sequence_preset":
            # Use predefined sequence
            sequence_name = commands.get("sequence_name")
            if sequence_name in PRESET_SEQUENCES:
                success = await execute_sequence(PRESET_SEQUENCES[sequence_name])
            else:
                success = False
                
        elif commands.get("command_type") == "sequence_dynamic":
            # Generate and execute dynamic sequence
            intent = commands.get("dynamic_intent", "be creative")
            style = commands.get("style", "normal")
            
            print(f"🎭 Generating dynamic sequence: '{intent}' with style '{style}'")
            dynamic_sequence = generate_dynamic_sequence(intent, style)
            success = await execute_sequence(dynamic_sequence)
            
        else:
            # Execute simple commands
            if commands.get("car_command") and commands["car_command"] != "S":
                success &= send_command(CAR_TOPIC, commands["car_command"], commands.get("speed"))
            
            if commands.get("command_type") == "position" and commands.get("positions"):
                success &= send_command(ARM_POSITION_TOPIC, "", positions=commands["positions"])
            elif commands.get("arm_command") and commands["arm_command"] != "S":
                success &= send_command(ARM_TOPIC, commands["arm_command"])
            
            if commands.get("duration", 0) > 0:
                asyncio.create_task(send_stop_after_delay(commands["duration"]))
        
        return {
            "success": success,
            "commands": commands,
            "timestamp": datetime.now().isoformat()
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/create-sequence")
async def create_custom_sequence(intent: str, style: str = "normal"):
    """Create a custom sequence for any intent"""
    try:
        sequence = generate_dynamic_sequence(intent, style)
        return {
            "success": True,
            "intent": intent,
            "style": style,
            "sequence": sequence,
            "steps": len(sequence),
            "timestamp": datetime.now().isoformat()
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/capabilities")
async def get_capabilities():
    """Get Alice's current capabilities"""
    return {
        "preset_sequences": list(PRESET_SEQUENCES.keys()),
        "dynamic_generation": True,
        "supported_styles": ["energetic", "gentle", "playful", "elegant", "dramatic", "normal"],
        "example_intents": [
            "be happy", "show excitement", "act like a cat", 
            "celebrate victory", "say goodbye", "stretch",
            "be creative", "surprise me", "welcome someone"
        ],
        "ai_model": "llama-3.1-8b-instant via Groq"
    }

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
        "message": "Alice emergency stop executed",
        "timestamp": datetime.now().isoformat()
    }

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "Alice is healthy and ready",
        "mqtt_connected": mqtt_client is not None,
        "ai_ready": True,
        "timestamp": datetime.now().isoformat()
    }

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)