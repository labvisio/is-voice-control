document.addEventListener('DOMContentLoaded', function() {
    const button = document.getElementById('mic-button');
    const transcriptionParagraph = document.getElementById('transcription');
    const feedbackParagraph = document.getElementById('feedback');

    let isActive = false;

    const socket = io();

    button.addEventListener('click', function() {
        if (!isActive) {
            button.textContent = "Desativar Microfone";
            button.classList.add('active');
            socket.emit('start_recording');  
        } else {
            button.textContent = "Ativar Microfone";
            button.classList.remove('active');
            socket.emit('stop_recording');  
        }
        isActive = !isActive;
    });

    socket.on('feedback', function(data) {
        feedbackParagraph.textContent = data.message;
    });

    socket.on('transcription', function(data) {
        transcriptionParagraph.textContent = `Você disse: ${data.text}`;
    });
});
