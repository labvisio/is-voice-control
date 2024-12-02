from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import speech_recognition as sr


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import speech_recognition as sr
import pyttsx3
import logging

from listener import Speaker_identification

class isVoiceControl:
    def __init__(self):
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'secret!'
        self.socketio = SocketIO(self.app)
        
        self.register_routes()

        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)d] - %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
            force=True,
        )
        self.engine = pyttsx3.init("espeak")
        self.sp = Speaker_identification(threshold=0.65)
        self.recognizer = sr.Recognizer()


    def register_routes(self):
        @self.app.route('/')
        def index():
            return render_template('index.html')
        
        @self.socketio.on('start_recording')
        def start_recording():
            with sr.Microphone() as source:
                emit('feedback', {'message': 'Microfone ativado. Escutando...'})
                while True:
                    self.sp.recognizer.adjust_for_ambient_noise(source, duration=1)
                    audio_operator = self.recognizer.listen(source)
                    try:  
                        command = self.recognizer.recognize_google(audio_operator, language='pt-BR').lower()
                        emit('transcription', {'text': command})
                        if 'operador' in command:
                            embedding = self.sp.processed_audio.collect_embedding(
                            audio_operator.get_wav_data()
                            )
                            operator_id = self.sp.verify_id(embedding, define_operador=True)
                            logging.info(
                                f"Envie um comando para o robô. ID = {operator_id}"
                            )
                            emit('transcription', {'text': f'Operador definido. ID = {operator_id}'})
                            self.operator = True
                    except sr.UnknownValueError:
                        emit('feedback', {'message': 'Não entendi o áudio. Tente novamente.'})
                    except sr.RequestError as e:
                        emit('feedback', {'message': f'Erro no serviço: {e}'})
        
                    if self.operator == True:
                        while True:
                            audio_command = self.recognizer.listen(source)
                            embedding = self.sp.processed_audio.collect_embedding(
                                audio_command.get_wav_data()
                            )
                            try:
                                command_text = self.recognizer.recognize_google(
                                    audio_command,
                                    language="pt-BR",
                                    ).lower()
                                command_id = self.sp.verify_id(
                                    embedding,
                                    define_operador=False,
                                    )
                            
                                if command_id is not None and command_id == operator_id:
                                    emit('transcription', {'text': command_text})
                                    logging.info(
                                        f"{command_text}"
                                        )
                                    
                                if 'deixar' in command_text:
                                    self.operator = False

                            
                                else:
                                    self.socketio.emit('transcription', {'text': 'Comando não autorizado. Operador não identificado.'})

                            except:
                                pass    

        @self.socketio.on('stop_recording')
        def stop_recording():
            self.socketio.emit('feedback', {'message': 'Microfone desativado. Parando gravação.'})
            self.operator = False

    def run(self):
        self.socketio.run(self.app, debug=True)

if __name__ == '__main__':
    app = isVoiceControl()
    app.run()
