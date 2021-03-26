const fs = require('fs');
const http = require('http');
const csv = require('fast-csv');
const express = require('express');
const socketIO = require('socket.io');
const bodyParser = require('body-parser');


const app = express();
const server = http.createServer(app);
const io = socketIO(server);
const port = process.env.PORT || 3000;

app.use(bodyParser.json());
app.use(express.static('public'));

const rcCsv = fs.createWriteStream('rcInput.csv', { flags: 'a' });
const airmarCsv = fs.createWriteStream('airmarOut.csv', { flags: 'a' });
const trimtabCsv = fs.createWriteStream('trimtabOut.csv', { flags: 'a' });

let countData = true;
let dataTimeDiff = 100;


app.post('/boat', (req, res) => {

	if (countData) {
		let data = req.body;
		countData = false;

		if (data.hasOwnProperty('latitude')){
			io.to('clients').emit('updateAirmarDash', req.body);
			addToDB(data, airmarCsv);
		} else if (data.hasOwnProperty('angle')) {
			io.to('clients').emit('updateTrimDash', req.body);
			addToDB(data, trimtabCsv);
		} else if (data.hasOwnProperty('rudder')) {
			io.to('clients').emit('updateSerialControls', req.body);
			addToDB(data, rcCsv);
		}

		setTimeout(() => countData = true, dataTimeDiff);
	}
	res.send(200);
});

// Inits Socket Connection from each client
io.on('connection', (socket) => {
	// If designated as a client, socket is sent to a room where the data will be emitted once recieved
	socket.on('client', () => socket.join('clients'));

	// once data is recieved emits to all clients in clients room
	socket.on('data', (data) => {
		io.to('clients').emit('updateDash', data);
		addToDB(data);
	});
});


// Function that will add to a database in the future
const addToDB = async (data, file) => {
	file.write('\n');
	csv.writeToStream(
	    	file,                  
			[[Date.now(), ...Object.values(data)]], 
			{headers:false}
	);
}

// addToDB({	'rate-of-turn': 6969, 
// 				'latitude': 50, 
// 				'latitude-direction': 50,
// 				'longitude': 50,
// 				'longitude-direction': 50,
// 				'track-degrees-true': 50,
// 				'track-degrees-magnetic': 50,
// 				'speed-knots': 50,
// 				'speed-kmh': 50,
// 				'outside-temp': 50,
// 				'atmospheric-pressure': 50,
// 				'magnetic-sensor-heading': 50,
// 				'magnetic-deviation': 50,
// 				'magnetic-deviation-direction': 50,
// 				'magnetic-variation': 50,
// 				'magnetic-variation-direction': 50,
// 				'wind-angle-true': 50,
// 				'wind-speed-true-knots': 50,
// 				'wind-speed-true-meters': 50,
// 				'wind-angle-relative': 50,
// 				'wind-speed-relative-meters': 50,
// 				'roll': 50,
// 				'pitch': 696969,
// 			}, airmarCsv);


// Server listen function
server.listen(port, () => console.log('Listening on port: ' + port));