from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import speech_recognition as sr

class MyFlaskApp:
    def __init__(self):
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'secret!'
        self.socketio = SocketIO(self.app)
        
        self.register_routes()

    def register_routes(self):
        @self.app.route('/')
        def index():
            return render_template('index.html')
        
        @self.socketio.on('start_recording')
        def start_recording():
            recognizer = sr.Recognizer()
            with sr.Microphone() as source:
                emit('feedback', {'message': 'Microfone ativado. Escutando...'})
                try:
                    while True:  
                        recognizer.adjust_for_ambient_noise(source, duration=1)
                        audio = recognizer.listen(source)
                        text = recognizer.recognize_google(audio, language='pt-BR').lower()
                        emit('transcription', {'text': text})
                except sr.UnknownValueError:
                    emit('feedback', {'message': 'Não entendi o áudio. Tente novamente.'})
                except sr.RequestError as e:
                    emit('feedback', {'message': f'Erro no serviço: {e}'})
        
        @self.socketio.on('stop_recording')
        def stop_recording():
            emit('feedback', {'message': 'Microfone desativado. Parando gravação.'})

    def run(self):
        self.socketio.run(self.app, debug=True)

if __name__ == '__main__':
    app = MyFlaskApp()
    app.run()
