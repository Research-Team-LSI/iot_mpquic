const WebSocket = require('ws');
const express = require('express');
const http = require('http');
const cors = require('cors');
const ExcelJS = require('exceljs');

const app = express();
app.use(cors());
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

// Array untuk menyimpan data sementara
let dataLog = [];

// Buat workbook dan worksheet sekali di awal
const workbook = new ExcelJS.Workbook();
const worksheet = workbook.addWorksheet('Data Log');

// Menambahkan header di worksheet
worksheet.columns = [
    { header: 'Timestamp', key: 'timestamp', width: 20 },
    { header: 'Temperature', key: 'temperature', width: 15 },
    { header: 'Humidity', key: 'humidity', width: 15 }
];

// Fungsi untuk menulis data ke file Excel setiap 10 menit
const writeToExcel = async () => {
    if (dataLog.length > 0) {  // Hanya tulis jika ada data baru
        dataLog.forEach((entry) => {
            worksheet.addRow(entry);
        });

        // Simpan workbook ke file Excel
        await workbook.xlsx.writeFile('data_log.xlsx');
        console.log('Data saved to data_log.xlsx');

        // Reset data log setelah menyimpan
        dataLog = [];
    } else {
        console.log('No new data to save.');
    }
};

// Update file Excel setiap 1 menit (60000 ms)
setInterval(writeToExcel, 60000);  // 1 menit

// Endpoint HTTP untuk menerima data dari ESP32
app.post('/data', express.json(), (req, res) => {
    const { temperature, humidity } = req.body;
    const timestamp = new Date().toLocaleString('en-GB', { timeZone: 'Asia/Jakarta' }); // Format waktu lengkap

    // Menyimpan data ke dalam array log dengan waktu lengkap
    dataLog.push({ timestamp, temperature, humidity });

    // Kirim data ke semua klien WebSocket
    wss.clients.forEach((client) => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(JSON.stringify({ temperature, humidity }));
        }
    });

    res.json({ status: 'success' });
});

// Untuk mengetahui kapan klien terhubung
wss.on('connection', (ws) => {
    console.log('Client connected');
});

// Jalankan server pada port 3000
server.listen(3000, () => {
    console.log('WebSocket server running on port 3000');
});
