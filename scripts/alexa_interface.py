#!/usr/bin/env python3

from flask import Flask
from flask_ngrok import run_with_ngrok
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter

from ask_sdk_core.dispatch_components import AbstractRequestHandler,AbstractExceptionHandler
from ask_sdk_core.utils import is_request_type, is_intent_name,request_util
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard

from alexa_ros_interface.my_turtlebot import MyTurtlebot
from alexa_ros_interface.integrate_ngrok import create_app
import rospy
import threading

# create `app` with ngrok setting
# Run os.environ["USE_NGROK"]="True" to disable Ngrok

app = create_app()

class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_request_type("LaunchRequest")(handler_input)
    
    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Hello, I'm your turtle bot"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Greeting", speech_text)).set_should_end_session(
            False)
        return handler_input.response_builder.response

class MoveIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("MoveIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        slots = handler_input.request_envelope.request.intent.slots
        location = slots['location'].value
        
        result = my_turtlebot.moveTo(location)
        
        speech_text = result
        
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Moving", speech_text)).set_should_end_session(
            False)
        
        return handler_input.response_builder.response

class GetPositionIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("GetPositionIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        
        position = my_turtlebot.getCurrentPosition()
        
        speech_text = "I am located at coordinates {:06.1f},{:06.1f}".format(position[0],position[1])
        
        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("My position", speech_text)).set_should_end_session(
            False)
        
        return handler_input.response_builder.response
    
class StopIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("StopIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok, I has stopped!"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Stop", speech_text)).set_should_end_session(
            False)
            
        my_turtlebot.stop()
    
        return handler_input.response_builder.response
    
class AllExceptionHandler(AbstractExceptionHandler):

    def can_handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> bool
        return True

    def handle(self, handler_input, exception):
        # type: (HandlerInput, Exception) -> Response

        speech = "Sorry, there was some problem. Please try again!!"
        handler_input.response_builder.speak(speech).ask(speech)
        
        return handler_input.response_builder.response
    
skill_builder = SkillBuilder()
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(MoveIntentHandler())
skill_builder.add_request_handler(StopIntentHandler())
skill_builder.add_request_handler(GetPositionIntentHandler())
skill_builder.add_exception_handler(AllExceptionHandler())

# Register your intent handlers to the skill_builder object
skill_adapter = SkillAdapter(
    skill=skill_builder.create(), skill_id="amzn1.ask.skill.e07485ae-5fcd-475a-90b2-93877974e388", app=app)

skill_adapter.register(app=app, route="/")

def createMyTurtlebot():
    global my_turtlebot 
    my_turtlebot = MyTurtlebot()
    rospy.spin()
    
if __name__ == '__main__':
   
   threading.Thread(target=createMyTurtlebot).start()
   
   app.run()