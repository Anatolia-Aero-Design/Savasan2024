from flask import Flask, request, jsonify
from Competition import Competition
import logging

app = Flask(__name__)

@app.route('/update_data', methods=['POST'])
def update_data():
    
    json_data = request.get_json()
    competition.update_contestant(request.get_json('takim_numarasi') , json_data)
    response = competition.response_json()
    print(response)
    
    return jsonify(response), 200

    

@app.route('/get_gps', methods=['GET'])
def post_gps():
    target_latitude = -35.361609
    target_longitude = 149.166369
    target_coordinates = {
        "target_latitude": target_latitude,
        "target_longitude": target_longitude
    }
    return jsonify(target_coordinates),200

if __name__ == '__main__':
    competition = Competition()
    app.run(debug=True, host='172.20.10.4', port=5000)
