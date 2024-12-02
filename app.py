from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import speech_recognition as sr

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('start_recording')
def start_recording():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        emit('feedback', {'message': 'Microfone ativado. Escutando...'})
        try:
            while True:  # Continua ouvindo até o cliente enviar um comando para parar
                recognizer.adjust_for_ambient_noise(source, duration=1)
                audio = recognizer.listen(source)
                text = recognizer.recognize_google(audio, language='pt-BR').lower()
                emit('transcription', {'text': text})
        except sr.UnknownValueError:
            emit('feedback', {'message': 'Não entendi o áudio. Tente novamente.'})
        except sr.RequestError as e:
            emit('feedback', {'message': f'Erro no serviço: {e}'})

@socketio.on('stop_recording')
def stop_recording():
    emit('feedback', {'message': 'Microfone desativado. Parando gravação.'})

if __name__ == '__main__':
    socketio.run(app, debug=True)
