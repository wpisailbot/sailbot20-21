const fs = require('fs');
const http = require('http');
// const csv = require('fast-csv');
const express = require('express');
const socketIO = require('socket.io');
const bodyParser = require('body-parser');
// const createCsvWriter = require('csv-writer').createObjectCsvWriter
const csvWriter = require('csv-write-stream');


const app = express();
const server = http.createServer(app);
const io = socketIO(server);
const port = process.env.PORT || 3000;

// faster this way trust me haha
const airmarHeaders = {'rate-of-turn': 0,'latitude': 1,'latitude-direction': 2,'longitude': 3,'longitude-direction': 4,'track-degrees-true': 5,'track-degrees-magnetic': 6,'speed-knots': 7,'speed-kmh': 8,'outside-temp': 9,'atmospheric-pressure': 10,'magnetic-sensor-heading': 11,'magnetic-deviation': 12,'magnetic-deviation-direction': 13,'magnetic-variation': 14,'magnetic-variation-direction': 15,'wind-angle-true': 16,'wind-speed-true-knots': 17,'wind-speed-true-meters': 18,'wind-angle-relative': 19,'wind-speed-relative-meters': 20,'roll': 21,'pitch': 22};
const trimtabHeaders = {'state': 0,'angle': 1};
const rcHeaders = {'state1': 0,'ballast': 1,'rudder': 2,'manual': 3,'state2': 4};

app.use(bodyParser.json());
app.use(express.static('public'));


let countData = true;
let dataTimeDiff = 100;
let writing = false;


app.post('/boat', (req, res) => {

	let data = req.body;
	if (countData && !writing) {
		countData = false;
		console.log('reading data:', data);
		if (data.hasOwnProperty('magnetic-sensor-heading') || data.hasOwnProperty('track-degrees-true')){
			writing = !writing;
			io.to('clients').emit('updateAirmarDash', req.body);
			// let airmarCsv = fs.createWriteStream('airmarOut.csv', { flags: 'a' });
			// addToDB(data, airmarCsv, airmarHeaders);
			addToDB(data, 'airmarOut.csv', airmarHeaders);
		} else if (data.hasOwnProperty('state')) {
			writing = !writing;
			io.to('clients').emit('updateTrimDash', req.body);
			// let trimtabCsv = fs.createWriteStream('trimtabOut.csv', { flags: 'a' });
			// addToDB(data, trimtabCsv, trimtabHeaders);
			addToDB(data, 'trimtabOut.csv', trimtabHeaders);
		} else if (data.hasOwnProperty('state1')) {
			writing = !writing;
			io.to('clients').emit('updateSerialControls', req.body);
			// rcCsv = fs.createWriteStream('rcInput.csv', { flags: 'a' });
			// addToDB(data, rcCsv, rcHeaders);
			addToDB(data, 'rcInput.csv', rcHeaders);
		}

		setTimeout(() => countData = true, dataTimeDiff);
	} else {
		console.log('trashed - ', 'count:', countData, 'wrting:', writing);
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
const addToDB = async (data, file, headers) => {
	let out = Object.keys(headers).map((val) => 0);
	Object.entries(data).forEach((entry, ind) => out[headers[entry[0]]] = entry[1]);
	fs.appendFile(file, ['\n' + getDateTime(), ...Object.values(out)], (err) => 
		{if(!err) writing = false});
}

const getDateTime = () => {
	let currentdate = new Date();
	return currentdate.getDate() + "/"+ (parseInt(currentdate.getMonth()) + 1)
	   + "/" + currentdate.getFullYear() + " "  
	   + currentdate.getHours() + ":"  
	   + currentdate.getMinutes() + ":" + currentdate.getSeconds();
}

// let airmarCsv = fs.createWriteStream('airmarOut.csv', { flags: 'a' });

// setTimeout(() => addToDB({
// 				'magnetic-sensor-heading': 50,
// 				'magnetic-deviation': 50,
// 				'magnetic-deviation-direction': 50,
// 				'magnetic-variation': 50,
// 				'magnetic-variation-direction': 50
// 			}, 'airmarOut.csv', airmarHeaders), 20);
// setTimeout(() => addToDB({
// 				'speed-knots': 50,
// 				'speed-kmh': 50,
// 				'outside-temp': 50,
// 				'atmospheric-pressure': 50
// 			}, 'airmarOut.csv', airmarHeaders), 21);
// setTimeout(() => addToDB({	'rate-of-turn': 6969, 
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
// 			}, 'airmarOut.csv', airmarHeaders), 21);



// Server listen function
server.listen(port, () => console.log('Listening on port: ' + port));