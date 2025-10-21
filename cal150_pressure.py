#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import threading
import struct
import minimalmodbus
from flask import Flask, request, render_template_string, jsonify
import serial

instrument = None

app = Flask(__name__)

######################################
# 1) Chercher un port USB valide     #
######################################
def find_instrument(baud=9600, addr=255):
    possible_ports = ['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyACM0']
    for p in possible_ports:
        try:
            instrument = minimalmodbus.Instrument(p, addr)
            instrument.serial.baudrate = baud
            instrument.serial.bytesize = 8
            instrument.serial.parity   = serial.PARITY_NONE
            instrument.serial.stopbits = 1
            instrument.mode            = minimalmodbus.MODE_RTU
            instrument.debug           = False
            instrument.byteorder = minimalmodbus.BYTEORDER_LITTLE_SWAP

            # Timeout & buffers
            instrument.clear_buffers_before_each_transaction = True
            instrument.serial.timeout = 0.5
            return instrument
        except:
            pass
    raise IOError("Impossible de trouver le CAL150 sur les ports testés.")

#######################################################
# 2) UI minimaliste pour définir une séquence de pression
#######################################################
HTML_PAGE = """
<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8" />
    <title>CAL150 Séquence</title>
</head>
<body>
<h2>CAL150 - Générer une séquence de pressions (mmHg)</h2>

<!-- Formulaire pour définir la séquence : liste type "10,60;20,60;30,60" -->
<p>Entrez une liste de paliers : <br/>
   Chaque palier : "valeurEnmmHg,duréeEnSecondes", séparés par point-virgules ;</p>
<form id="seqForm">
  <textarea id="sequenceInput" rows="3" cols="50">10,5;20,5;30,5;40,5;50,5;60,5</textarea><br/>
  <button type="submit">Démarrer la séquence</button>
</form>

<div id="status"></div>

<script>
document.getElementById('seqForm').addEventListener('submit', function(e){
    e.preventDefault();
    let seqText = document.getElementById('sequenceInput').value;
    fetch('/start_sequence', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ sequence: seqText })
    })
    .then(resp => resp.text())
    .then(txt => {
      document.getElementById('status').innerText = txt;
      alert(txt);
    })
    .catch(err => alert("Erreur : " + err));
});
</script>
</body>
</html>
"""

@app.route('/')
def index():
    return HTML_PAGE

#######################################################
# 3) Démarrer la séquence : parse, convertit mmHg → Pa
#######################################################
sequence_thread = None
is_running = False

@app.route('/start_sequence', methods=['POST'])
def start_sequence():
    global sequence_thread, is_running

    try:
        instrument = find_instrument()
        print("Instrument trouvé.")
    except Exception as e:
        print("ERREUR : ", e)

    if not instrument:
        return "Erreur: Aucun instrument détecté.", 500

    data = request.get_json()
    seq_str = data.get('sequence','').strip()
    if not seq_str:
        return "Aucune séquence définie.", 400

    # Parse la séquence : "10,60;20,60;30,60"
    steps = []
    try:
        for chunk in seq_str.split(';'):
            chunk = chunk.strip()
            if not chunk:
                continue
            mmHg_str, dur_str = chunk.split(',')
            mmHg = float(mmHg_str)
            dur  = float(dur_str)
            steps.append((mmHg, dur))
        
        if is_running:
            return "Une séquence est déjà en cours.", 400

        print("Instrument port:", instrument.serial.port)

        print(steps)
        try:
            for mmHg, duration in steps:
                valeur_pa = mmHg * 133.322
                instrument.write_float(154, valeur_pa)
                #time.sleep(0.2)
                instrument.write_register(138, 16384, functioncode=6)
                print(f"Palier {mmHg} mmHg -> {valeur_pa} Pa pour {duration}s")
                time.sleep(duration)
            # A la fin
            instrument.write_register(138, 0, functioncode=6)
            is_running = False
            return "Séquence terminée en mode synchrone", 200
        except Exception as e:
            return f"Erreur: {e}", 500

    except:
        return "Format invalide. Exemple: 10,5;20,10;30,5", 400

    
    
    # Lancer un thread pour exécuter la séquence sans bloquer Flask
    #sequence_thread = threading.Thread(target=run_sequence, args=(steps,))
    #sequence_thread.start()
    #return f"Séquence démarrée : {steps}", 200

########################################
# 4) Exécuter la séquence en thread
########################################
def run_sequence(steps):
    global is_running
    is_running = True
    try:
        for (mmHg, duration) in steps:
            # Conversion mmHg -> Pa
            value_pa = mmHg * 133.322

            # 1) Écrire la consigne float dans le registre 154 (function 16)
            #    Attention : write_float() peut nécessiter la version récente de minimalmodbus
            #    Sinon, on peut faire un write_registers après conversion manuelle en 2 words.
            #try:
                # Selon la version de minimalmodbus, passer en positionnel :
            instrument.write_float(154, value_pa)
            #except TypeError:
                # Si "functioncode" n'est pas supporté en kwarg, on tente en positionnel
            #    instrument.write_float(154, value_pa, 2, 16)

            # 2) Lancer la pression à 100% (reg. 600139 = 16384)
            instrument.write_register(138, 16384, functioncode=6)
            print(f"Palier {mmHg} mmHg (≈ {value_pa:.1f} Pa) lancé pour {duration}s")

            # 3) Attendre la durée spécifiée
            time.sleep(duration)

        # Fin de séquence => Stop (0%)
        instrument.write_register(138, 0, functioncode=6)
        print("Séquence terminée, pression relâchée à 0%")

    except Exception as e:
        print("Erreur dans la séquence :", e)
    finally:
        is_running = False

#########################################################
# Lancement du serveur Flask
#########################################################
if __name__ == '__main__':
    # Si l’instrument n’est pas trouvé, on peut quand même lancer Flask,
    # mais /start_sequence échouera
    app.run(host='0.0.0.0', port=5100, debug=False)
