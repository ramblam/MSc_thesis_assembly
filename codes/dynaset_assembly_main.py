import paho.mqtt.client as mqtt
import json
import socket
from datetime import datetime
import threading
import time
import os
import rospy
from std_msgs.msg import String
#Chose audiofile location based on operating system.
audioPath = "/usr/share/creoir/"

lastAction = ""

pub=rospy.Publisher("/text_commands", String, queue_size=10)
sub_listen = True
wakeup_counter = 0
task = 1
did_you_mean_tool = ""

#Boilerplate connect callback function.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("creoir/#", 0)

#Callback funtion to "creoir/asr/intentRecognized" topic.
def on_message_intent(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload.decode("utf-8")))
    dynaset_assembly_eventloop(msg.payload, msg.topic)
    #This avoids going back to WUW:
    formatPayload = {"contextName":'MAIN_OVS', "timeOut":10000}
    json_dump = json.dumps(formatPayload)
    client.publish("creoir/asr/setContext", json_dump)

#Callback function to "creoir/asr/listening" topic.
def on_message_wuw(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload.decode("utf-8")))
    dynaset_assembly_eventloop(msg.payload, msg.topic)

#Callback function to "creoir/asr/intentNotRecognized" topic.
def on_message_timeout(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload.decode("utf-8")))
    sub_listen = False
    dynaset_assembly_eventloop(msg.payload, msg.topic)

#Callback function to "creoir/device/versioninfo" topic.
def on_message_version(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload.decode("utf-8")))
    dynaset_assembly_eventloop(msg.payload, msg.topic)

#Callback function to "creoir/config/current" topic.
def on_message_config(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload.decode("utf-8")))

#MQTT handler function. This function in target for the thread created in the main.
def mqttHandler():
    client.message_callback_add('creoir/device/versioninfo', on_message_version)
    client.message_callback_add('creoir/asr/intentRecognized', on_message_intent)
    client.message_callback_add('creoir/asr/listening', on_message_wuw)
    client.message_callback_add('creoir/asr/intentNotRecognized', on_message_timeout)
    client.message_callback_add('creoir/config/current', on_message_config)

    client.on_connect = on_connect
    client.connect("localhost", 1883, 60)
    client.loop_forever()

#Function used to set the OVS parameters manually.
def setOvsParameters():
    config = {}
    config["utteranceThreshold"] = 4500
    config["wakewordThreshold"] = 4200
    config["slotThreshold"] = 0
    config["sendTestEvents"] = 0
    config["silenceTimeout"] = 10000
    config["babbleTimeout"] = 0
    payload = json.dumps(config)
    client.publish("creoir/config/set", payload)

def handOver(objectName, jsonData):
        print(objectName)
        print(jsonData)
        pub.publish("GIVE "+objectName.upper())

def ros_to_creoir_tts_callback(msg):
    msg = msg.data
    global did_you_mean_tool
    if sub_listen:
        if msg == "Can not hand over screwdriver as it was not detected. Did you mean allen key?":
            TTS_SayUtteranceMQTT(msg)
            did_you_mean_tool = "allen key"
            switchContext("YESNO", 10000)
        elif msg == "Can not hand over allen key as it was not detected. Did you mean screwdriver?":
            TTS_SayUtteranceMQTT(msg)
            did_you_mean_tool = "screwdriver"
            switchContext("YESNO", 10000)
        elif msg =="setContextMain":
            #This is used for extending the timeout if it runs out during robot actions.
            formatPayload = {"contextName":'MAIN_OVS', "timeOut":10000}
            json_dump = json.dumps(formatPayload)
            client.publish("creoir/asr/setContext", json_dump)
        else:
        	TTS_SayUtteranceMQTT(msg)
    
sub = rospy.Subscriber("/ros_to_creoir_tts", String, ros_to_creoir_tts_callback)

def placeObject(objectName, jsonData):
        print(objectName)
        print(jsonData)
        pub.publish("PLACE "+objectName.upper())
        
def pickObject(objectName, jsonData):
        print(objectName)
        print(jsonData)
        pub.publish("PICK "+objectName.upper())
        
def assembleObject(objectName, jsonData):
        print(objectName)
        print(jsonData)
        pub.publish("ASSEMBLE "+objectName.upper())
        
def setMode(modeName, jsonData):
        print(modeName)
        print(jsonData)
        pub.publish("MODE "+modeName.upper())
        
def moveCommand(direction, jsonData):
        print(direction)
        print(jsonData)
        pub.publish("MOVE "+direction.upper())
        
def moveToPosition(position, jsonData):
        print(position)
        print(jsonData)
        pub.publish("POSITION "+position.upper())
        
def moveAbove(position, distance, unit, jsonData):
        print(position)
        print(distance)
        print(jsonData)
        pub.publish("HOLD "+position.upper()+" DISTANCE "+distance+" "+unit.upper())
        
def setStepSize(stepSize, jsonData):
        print(stepSize)
        print(jsonData)
        pub.publish("STEP SIZE "+stepSize.upper())
        
#Utility function used to find computers current ip-address. This ip will be returned and then spoken by tts.
def findMyIPaddress():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0].split(".",3)
        s.close()
        return 0, ip
    except socket.error as exc:
        print(f"Socket error: {exc}")
        return 1, "No internet connection"

#Utility function used for controlling the TTS. Inputs: String payload str payload_type which determines if the payload is a file or a text.
def TTS_SayUtteranceMQTT(payload, payload_type="text"):
    if "wav" not in payload_type:
        formatPayload = {"utterance":payload}
        json_dump = json.dumps(formatPayload)
        client.publish("creoir/talk/speak", json_dump)
    else:
        formatPayload = {"file":audioPath + payload}
        json_dump = json.dumps(formatPayload)
        client.publish("creoir/talk/speak", json_dump)

#Utility function used to simplify the eventloop and move functionality out of it.
#This function is used to change the grammar of the OVS. Takes grammar name and desired timeout as input.
def switchContext(grammar, timeout):
    formatPayload = {"contextName":grammar, "timeOut":timeout}
    json_dump = json.dumps(formatPayload)
    client.publish("creoir/asr/setContext", json_dump)

#Eventloop. This function is in charge of parsing the correct intent from the mqtt message.
#It then either takes the action or passes the message to utility function for further parsing.
#This function is called directly from the mqtt callback functions.
#Takes mqtt message and topic as an input.
def dynaset_assembly_eventloop(msg, topic):
    global lastAction
    global task
    global wakeup_counter
    global did_you_mean_tool
    print("msg here: ", msg)
    jsonData = json.loads(msg)
    if "intentRecognized" not in topic:
        if "listening" in topic:
            TTS_SayUtteranceMQTT("wakeup.wav", "wav")
            if wakeup_counter==0 and task == 1:
            	TTS_SayUtteranceMQTT("Hello, Let's start by assembling cover")
            	wakeup_counter +=1
            elif wakeup_counter==0 and task == 2:
            	TTS_SayUtteranceMQTT("Hello, in task two, let's assemble the elbow")
            	wakeup_counter +=1
            elif wakeup_counter==0 and task == 3:
            	TTS_SayUtteranceMQTT("Hello, in task three, I can help you with the pipe")
            	wakeup_counter +=1
        elif "intentNotRecognized" in topic:
            TTS_SayUtteranceMQTT("low_confidence.wav", "wav")
    else:
        if "HAND_OVER" in jsonData["intent"]:
        	if len(jsonData["slots"]) != 0:
        		if "object" in jsonData["slots"][0]["slotName"]:
        			objectName = jsonData["slots"][0]["slotValue"]
        			handOver(objectName, jsonData)
        			return
        	lastAction = jsonData

        elif "PLACE_OBJECT" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                if "object" in jsonData["slots"][0]["slotName"]:
                    objectName = jsonData["slots"][0]["slotValue"]
                    placeObject(objectName, jsonData)
                    return

            lastAction = jsonData
            
        elif "PAUSE" in jsonData["intent"]:
        	pub.publish("PAUSE")
        	lastAction = jsonData

        elif "GIVE_INSTRUCTIONS" in jsonData["intent"]:
            if task == 1:
                formatTTS = ("In task one, I can assemble the cover, give you the big bolts and screwdriver. You need to screw the bolts")
            elif task == 2:
                formatTTS = ("In task two, I can assemble the elbow and give you the allen key")
            else:
                formatTTS = ("In task three, I can pick the pipe and hold it still for you")
            TTS_SayUtteranceMQTT(formatTTS)
            lastAction = jsonData
            
        elif "PICK_OBJECT" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                if "object" in jsonData["slots"][0]["slotName"]:
                    objectName = jsonData["slots"][0]["slotValue"]
                    pickObject(objectName, jsonData)
                    return

            lastAction = jsonData
            
        elif "QUALITY_CHECK" in jsonData["intent"]:
        	pub.publish("QUALITY CHECK")
        	lastAction = jsonData
            
        elif "ASSEMBLE_OBJECT" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                if "directly_placeable" in jsonData["slots"][0]["slotName"]:
                    objectName = jsonData["slots"][0]["slotValue"]
                    assembleObject(objectName, jsonData)
                    return

            lastAction = jsonData
            
        elif "GO_HOME" in jsonData["intent"]:
        	pub.publish("MOVE HOME")
        	lastAction = jsonData

        elif "MOVE_DIST_UNIT_DIR" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                for i in range(len(jsonData["slots"])):
                    if "OVS_number" in jsonData["slots"][i]["slotName"]:
                        distance = jsonData["slots"][i]["slotValue"]
                    elif "distance" in jsonData["slots"][i]["slotName"]:
                        distance = jsonData["slots"][i]["slotValue"]
                    elif "unit" in jsonData["slots"][i]["slotName"]:
                        unit = jsonData["slots"][i]["slotValue"]
                    elif "direction" in jsonData["slots"][i]["slotName"]:
                        direction = jsonData["slots"][i]["slotValue"]
                pub.publish("MOVE "+direction.upper()+" "+distance+" "+unit.upper())

            lastAction = jsonData
            
        elif "CONTINUE" in jsonData["intent"]:
        	pub.publish("CONTINUE")
        	lastAction = jsonData
            
        elif "TOOL_OPEN" in jsonData["intent"]:
        	pub.publish("TOOL OPEN")
        	lastAction = jsonData
            
        elif "TOOL_CLOSE" in jsonData["intent"]:
        	pub.publish("TOOL CLOSE")
        	lastAction = jsonData
            
        elif "TOOL_ROTATE" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                for i in range(len(jsonData["slots"])):
                    if "rotation_direction" in jsonData["slots"][i]["slotName"]:
                    	rot_dir = jsonData["slots"][i]["slotValue"]
                pub.publish("TOOL ROTATE "+rot_dir.upper())
                lastAction = jsonData
            
        elif "SET_MODE" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                if "mode" in jsonData["slots"][0]["slotName"]:
                    modeName = jsonData["slots"][0]["slotValue"]
                    setMode(modeName, jsonData)
                    return

            lastAction = jsonData
            
        elif "MOVE_TO" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                if "position" in jsonData["slots"][0]["slotName"]:
                    position = jsonData["slots"][0]["slotValue"]
                    moveToPosition(position, jsonData)
                    return

            lastAction = jsonData
            
        elif "MOVE_ABOVE" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                for i in range(len(jsonData["slots"])):
                    if "OVS_number" in jsonData["slots"][i]["slotName"]:
                        distance = jsonData["slots"][i]["slotValue"]
                    elif "distance" in jsonData["slots"][i]["slotName"]:
                        distance = jsonData["slots"][i]["slotValue"]
                    elif "unit" in jsonData["slots"][i]["slotName"]:
                        unit = jsonData["slots"][i]["slotValue"]
                    elif "position" in jsonData["slots"][i]["slotName"]:
                    	position = jsonData["slots"][i]["slotValue"]
                moveAbove(position, distance, unit, jsonData)

            lastAction = jsonData
            
        elif "MOVE" in jsonData["intent"]:
        	if len(jsonData["slots"]) != 0:
        		if "direction" in jsonData["slots"][0]["slotName"]:
        			direction = jsonData["slots"][0]["slotValue"]
        			moveCommand(direction, jsonData)
        			return
        	lastAction = jsonData
            
        elif "SET_STEP_SIZE" in jsonData["intent"]:
            if len(jsonData["slots"]) != 0:
                if "step_size" in jsonData["slots"][0]["slotName"]:
                    step_size = jsonData["slots"][0]["slotValue"]
                    setStepSize(step_size, jsonData)
                    return

            lastAction = jsonData
        
        elif "ANSWER_YES" in jsonData["intent"]:
        	if did_you_mean_tool == "screwdriver":
        		handOver("screwdriver", jsonData)
        		did_you_mean_tool = ""
        		return
        	elif did_you_mean_tool == "allen key":
        		handOver("allen key", jsonData)
        		did_you_mean_tool = ""
        		return
        elif "ANSWER_NO" in jsonData["intent"]:
            did_you_mean_tool = ""
            return
        
        else:
            return

#The main function of the code. It initializes the MQTT client, starts the MQTT handle thread,
#Gets the current configuration and changes that configuration.
#It then enters sleep loop, giving the thread function all of the processing time.
if __name__ == "__main__":
    try:
        client = mqtt.Client()
        t1 = threading.Thread(target=mqttHandler, args=())
        t1.start()

        time.sleep(1)
        client.publish("creoir/config/get", payload=None)
        time.sleep(2)
        setOvsParameters()
        client.publish("creoir/config/get", payload=None)
        
        rospy.init_node('text_command_publisher_creoir')
        rospy.spin()

        while(1):
            time.sleep(1)
    except KeyboardInterrupt:
        os._exit(0)
