const http = require('http');
const express = require('express');
const socketIO = require('socket.io');
const bodyParser = require('body-parser');
const { MongoClient } = require("mongodb");

const app = express();
const server = http.createServer(app);
const io = socketIO(server);
const port = process.env.PORT || 3000;

const url = "mongodb+srv://sailbothot:admin@cluster0.ay01x.mongodb.net/myFirstDatabase?retryWrites=true&w=majority&useNewUrlParser=true&useUnifiedTopology=true";
const client = new MongoClient(url);

app.use(bodyParser.json());
app.use(express.static('public'));

let countData = true;
let dataTimeDiff = 0;

let collection = undefined;
const dbName = "Sailbot";


app.post('/boat', (req, res) => {

	if (countData) {
		let data = req.body;
		countData = false;

		if (data.hasOwnProperty('latitude'))
			io.to('clients').emit('updateAirmarDash', req.body);
		else if (data.hasOwnProperty('angle'))
			io.to('clients').emit('updateTrimDash', req.body);
		else if (data.hasOwnProperty('rudder'))
			io.to('clients').emit('updateSerialControls', req.body);

		addToDB(data);
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

const startDB = async () => {
	await client.connect();
	const db = client.db(dbName);
	collection = db.collection("TestData");     
	console.log("Connected correctly to collection");
}

// Function that will add to a database in the future
const addToDB = async (data) => {
	db.push(data); // implement real online and offline database eventually
	await col.insertOne(data);
}

startDB();
// Server listen function
server.listen(port, () => console.log('Listening on port: ' + port));