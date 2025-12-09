const compassSlider = document.getElementById('compassSlider'); // Get the compass slider element from the HTML page

const STATUS_INTERVAL = 10000; // 10 seconds

checkStatus();
fetchLogs();

const input = document.getElementById('numInput');
let speed_value = 50;
const msg = document.getElementById('msg');

input.addEventListener('input', () => {
    speed_value = Number(input.value);
    if (speed_value < 1 || speed_value > 100 || speed_value.value === "") {
		speed_value = 50;
        msg.classList.remove('hidden');
    } else {
        msg.classList.add('hidden');
    }
});

setInterval(() => {
	checkStatus();
	fetchLogs();
}, STATUS_INTERVAL,);

function checkStatus() {
	fetch('/status')
		.then(res => {
			if (!res.ok) throw new Error("Status request failed");
			return res.text();   // assume server returns "true" or "false"
		})
		.then(data => {
			const isActive = (data.trim().toLowerCase() === "true");
			console.log("Status " + isActive + " received.");
			setConnectionState(isActive);   // update your status label
		})
		.catch(err => {
			console.log("Status false received.");
			console.error("Status error:", err);
			setConnectionState(false);       // treat failures as NOT ACTIVE
		});
}


function fetchLogs() {
	fetch('/log')
		.then(res => {
			if (!res.ok) throw new Error("Log request failed");
			return res.text();
		})
		.then(text => {
			if (text && text.trim().length > 0) {
				const logsInput = document.getElementById('logs');
				if (logsInput) logsInput.value = text;
			}
		})
		.catch(err => {
			console.warn("Log fetch failed; keeping old logs:", err);
			// Do nothing – keep old value
		});
}

function setConnectionState(isActive) {
	const wrapper = document.getElementById('connection');
	const text = wrapper.querySelector('.status-text');

	if (isActive) {
		wrapper.dataset.state = "active";
		text.textContent = "ACTIVE";
	} else {
		wrapper.dataset.state = "inactive";
		text.textContent = "NOT ACTIVE";
	}
}

// Check if the compassSlider element exists and reset it to 0
if (compassSlider) {
	const middleValue = 0;
	compassSlider.value = middleValue;	// Set slider to the middle position (0 degrees)
	updateCompass(middleValue);			// Set the compass value to initial value of 0	
}


// Function to update the compass display (show the current compass position)
function updateCompass(pos) {
	document.getElementById('compassValue').innerText = `${pos}°`;// Update the text value for compassValue when value is changing on page element with the current position and add "°" for degrees
}


// Function to send the current compass value to the server
function sendCompassValue(pos) {
	fetch(`/compass?value=${pos}`);		// Send the compass value to the server using a fetch request
	console.log("Compass value", pos);	// Log the compass value to the console for debugging
}


// Functions for moving the motor forward and backward with specific distances
function forwards5() { move('forwards', 5); } 		// Calling move function with separated parameters
function forwards20() { move('forwards', 20); }		// Calling move function with separated parameters
function backwards5() { move('backwards', 5); }		// Calling move function with separated parameters
function backwards20() { move('backwards', 20); }	// Calling move function with separated parameters
function findNorth() { find('north'); } // Calling find function with parameter north

// This is a general-purpose function that can handle different combinations of direction and distance
function move(dir, dis) {
	fetch(`/${dir}${dis}?speed=${speed_value}`);			// Sends a request to the server with a URL constructed from the received direction and distance (e.g., /forwards5)
	console.log("Drive", dir, dis);	// Log the movement command to the console for debugging
}

// This is a function used to handle find cardinal direction command
function find(cardinalDir) {
	fetch(`/find${cardinalDir}`);
	console.log("Find", cardinalDir);
}


