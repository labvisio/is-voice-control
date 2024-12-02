document.addEventListener('DOMContentLoaded', function() {
    const button = document.getElementById('mic-button');
    const transcriptionParagraph = document.getElementById('transcription');

    let isActive = false;

    button.addEventListener('click', function() {
        if (!isActive) {
            button.textContent = "Desativar Microfone";
            button.classList.add('active');
            startRecording();
        } else {
            button.textContent = "Ativar Microfone";
            button.classList.remove('active');
        }
        isActive = !isActive;
    });

    function startRecording() {
        fetch('/record', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                if (data.transcription) {
                    transcriptionParagraph.textContent = `Você disse: ${data.transcription}`;
                } else if (data.error) {
                    transcriptionParagraph.textContent = `Erro: ${data.error}`;
                }
            });
    }
});
