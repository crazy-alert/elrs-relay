// script.js - Common functionality for ELRS Repeater Pro

// Глобальные переменные состояния
let refreshInterval = null;
let currentTab = 'dashboard';

// API запросы с обработкой ошибок
async function apiRequest(url, options = {}) {
    try {
        const response = await fetch(url, options);
        if (!response.ok) {
            throw new Error(`HTTP error ${response.status}`);
        }
        return await response.json();
    } catch (error) {
        console.error('API Error:', error);
        return null;
    }
}

// Обновление статуса (Dashboard)
async function fetchStatus() {
    const data = await apiRequest('/api/status');
    if (!data) return;
    
    updateElement('linkRadio', data.linkRadio ? '✅ CONNECTED' : '❌ DISCONNECTED');
    updateElement('linkRadio', data.linkRadio ? '✅ CONNECTED' : '❌ DISCONNECTED', 'className', data.linkRadio ? 'status-ok' : 'status-bad');
    updateElement('linkCopter', data.linkCopter ? '✅ CONNECTED' : '❌ DISCONNECTED');
    updateElement('linkCopter', data.linkCopter ? '✅ CONNECTED' : '❌ DISCONNECTED', 'className', data.linkCopter ? 'status-ok' : 'status-bad');
    updateElement('rssi', data.rssi || '---');
    updateElement('packets', data.packets);
    updateElement('packetsBad', data.packetsBad);
    updateElement('packetsGood', data.packetsGood);
    updateElement('baudrate', data.baudrate);
    updateElement('deviceName', data.deviceName);
    updateElement('txpower', data.txpower + ' mW');
    updateElement('pktrate', data.pktrate + ' Hz');
}

// Обновление статуса питания
async function fetchPowerStatus() {
    const data = await apiRequest('/api/powerMode');
    if (!data) return;
    
    updateElement('relayState', data.relayState ? '✅ ON' : '❌ OFF');
    const modeText = data.mode === 0 ? 'Auto (Bind Only)' : (data.mode === 1 ? 'Always ON' : 'Always OFF');
    updateElement('powerModeText', modeText);
    
    if (document.getElementById('modeAuto')) {
        document.getElementById('modeAuto').checked = (data.mode === 0);
        document.getElementById('modeAlwaysOn').checked = (data.mode === 1);
        document.getElementById('modeAlwaysOff').checked = (data.mode === 2);
    }
}

// Вспомогательная функция обновления DOM-элемента
function updateElement(id, value, attr = 'innerHTML') {
    const el = document.getElementById(id);
    if (el) {
        if (attr === 'className') el.className = value;
        else el.innerHTML = value;
    }
}

// Переключение вкладок
function switchTab(tabId) {
    // Скрыть все вкладки
    document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
    document.querySelectorAll('.tab-btn').forEach(el => el.classList.remove('active'));
    
    // Показать выбранную
    const tabContent = document.getElementById(`tab-${tabId}`);
    if (tabContent) tabContent.classList.add('active');
    const tabBtn = document.querySelector(`.tab-btn[data-tab="${tabId}"]`);
    if (tabBtn) tabBtn.classList.add('active');
    
    currentTab = tabId;
    
    // Специфичные действия при переключении
    if (tabId === 'transmitter' && typeof loadTxMenu === 'function') {
        loadTxMenu(0);
    } else if (tabId === 'files' && typeof listFiles === 'function') {
        listFiles();
    }
}

// Запуск периодического обновления
function startAutoRefresh(intervalMs = 1000) {
    if (refreshInterval) clearInterval(refreshInterval);
    refreshInterval = setInterval(() => {
        if (currentTab === 'dashboard') {
            fetchStatus();
            fetchPowerStatus();
        }
        // Для других вкладок обновление по требованию
    }, intervalMs);
}

// При загрузке страницы
document.addEventListener('DOMContentLoaded', () => {
    fetchStatus();
    fetchPowerStatus();
    startAutoRefresh(1000);
    
    // Привязка обработчиков кнопок, если они есть на странице
    const applyPowerBtn = document.getElementById('applyPowerMode');
    if (applyPowerBtn) {
        applyPowerBtn.addEventListener('click', applyPowerMode);
    }
});

// Функции управления питанием (пример)
async function applyPowerMode() {
    const auto = document.getElementById('modeAuto')?.checked;
    const alwaysOn = document.getElementById('modeAlwaysOn')?.checked;
    const mode = auto ? 0 : (alwaysOn ? 1 : 2);
    await apiRequest(`/api/setPowerMode?mode=${mode}`);
    fetchPowerStatus();
}

// Установка baudrate
async function setBaudrate() {
    const baudInput = document.getElementById('newBaud');
    if (!baudInput) return;
    const baud = baudInput.value;
    if (confirm(`Change baudrate to ${baud} and restart?`)) {
        await fetch(`/api/setBaud?baud=${baud}`);
        setTimeout(() => location.reload(), 2000);
    }
}