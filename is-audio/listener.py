import speech_recognition as sr
import logging
from audiorecognizer import Extract_embeddings
from scipy.spatial.distance import cosine
import numpy as np


class Speaker_identification:
    def __init__(self, threshold=0.65):
        logging.basicConfig(
            level=logging.DEBUG,
            format="%(asctime)s - %(levelname)s - [%(filename)s:%(lineno)d] - %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
            force=True,
        )
        self.data_audio = {}
        self.id = 0
        self.operator = False
        self.threshold = threshold
        self.recognizer = sr.Recognizer()
        self.processed_audio = Extract_embeddings()

    def verify_id(self, embedding, define_operador=False):
        embedding = embedding[0]
        max_distance = float("inf")
        if not self.data_audio:
            self.data_audio[self.id] = embedding
            return self.id
        for class_id, class_embedding in self.data_audio.items():
            distance = cosine(embedding, class_embedding)
            if distance < max_distance:
                max_distance = distance
                current_id = class_id
            if max_distance < self.threshold:
                return current_id
        if define_operador:
            self.id += 1
            self.data_audio[self.id] = embedding
            logging.info(f"New id assigned: {self.id}")
            return self.id
        return None
