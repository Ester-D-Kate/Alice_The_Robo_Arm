// Helper to send commands to ESP8266
function sendCommand(cmd, type = 'movement') {
    let url = '/button?btn=' + encodeURIComponent(cmd);
    if (type === 'speed') {
        url = '/speed?value=' + encodeURIComponent(cmd);
    }
    fetch(url);
}

function sendRelease() {
    fetch('/button_release');
}

// Joystick logic
const joystick = document.getElementById('joystick');
const knob = document.getElementById('joystickKnob');
let dragging = false;
let center = { x: 0, y: 0 };
let maxDist = 0;
let lastDir = "S";
let carSpeed = 50; // Default speed

// Speed slider functionality
const speedSlider = document.getElementById('speedSlider');
const speedValue = document.getElementById('speedValue');

// Update speed value display and send speed command
speedSlider.addEventListener('input', function() {
    carSpeed = speedSlider.value;
    speedValue.textContent = carSpeed + '%';
    sendCommand(carSpeed, 'speed');
});

// Add touch support for mobile devices
speedSlider.addEventListener('touchstart', function(e) {
    e.stopPropagation(); // Prevent other touch events from firing
});

function getDirection(dx, dy) {
    const angle = Math.atan2(dy, dx) * 180 / Math.PI;
    if (Math.sqrt(dx*dx + dy*dy) < maxDist * 0.3) return "S"; // Center = Stop
    if (angle >= -45 && angle < 45) return "R";
    if (angle >= 45 && angle < 135) return "B";
    if (angle >= -135 && angle < -45) return "F";
    return "L";
}

function setKnob(x, y) {
    knob.style.left = x + 'px';
    knob.style.top = y + 'px';
}

function resetKnob() {
    setKnob(joystick.clientWidth/2, joystick.clientHeight/2);
}

joystick.addEventListener('pointerdown', function(e) {
    dragging = true;
    const rect = joystick.getBoundingClientRect();
    center = { x: rect.width/2, y: rect.height/2 };
    maxDist = rect.width/2 - knob.offsetWidth/2;
    moveKnob(e);
});

window.addEventListener('pointermove', function(e) {
    if (!dragging) return;
    moveKnob(e);
});

window.addEventListener('pointerup', function(e) {
    if (!dragging) return;
    dragging = false;
    resetKnob();
    lastDir = "S";
    sendCommand("S");
});

function moveKnob(e) {
    const rect = joystick.getBoundingClientRect();
    let x = e.clientX - rect.left;
    let y = e.clientY - rect.top;
    let dx = x - center.x;
    let dy = y - center.y;
    const dist = Math.sqrt(dx*dx + dy*dy);
    if (dist > maxDist) {
        dx = dx * maxDist / dist;
        dy = dy * maxDist / dist;
    }
    setKnob(center.x + dx, center.y + dy);
    const dir = getDirection(dx, dy);
    if (dir !== lastDir) {
        lastDir = dir;
        sendCommand(dir);
    }
}

// Initialize knob position
resetKnob();

// Add tactile effect for all buttons on pointer events and send commands
document.addEventListener('DOMContentLoaded', function () {
    document.querySelectorAll('button').forEach(function (btn) {
        btn.addEventListener('pointerdown', function () {
            btn.classList.add('pressed');
            // Map button to command
            let cmd = "";
            if (btn.id === "Up") cmd = "U";
            else if (btn.id === "Down") cmd = "D";
            else if (btn.id === "Close") cmd = "C";
            else if (btn.id === "Open") cmd = "O";
            else if (btn.id === "extra1" || btn.className === "extra1") cmd = "1";
            else if (btn.id === "extra2" || btn.className === "extra2") cmd = "2";
            else if (btn.id === "extra3" || btn.className === "extra3") cmd = "3";
            else if (btn.id === "extra4" || btn.className === "extra4") cmd = "4";
            else if (btn.classList.contains("Stop")) cmd = "S";
            else cmd = btn.textContent.trim();
            sendCommand(cmd);
        });
        btn.addEventListener('pointerup', function () {
            btn.classList.remove('pressed');
            sendRelease();
        });
        btn.addEventListener('pointerleave', function () {
            btn.classList.remove('pressed');
            sendRelease();
        });
    });
});

// Theme toggle logic
const themeToggle = document.getElementById('themeToggle');
themeToggle.addEventListener('click', function () {
    document.body.classList.toggle('dark');
    // Change icon
    themeToggle.textContent = document.body.classList.contains('dark') ? '-O-' : '-D-';
});
