/**
 * @file WebUI.cpp
 * @brief Embedded HTTP server for recipe CRUD and a tiny UI.
 */
#include "app/WebUI.hpp"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WiFi.h>

#include "util/Recipe.hpp"
#include "util/Storage.hpp"

namespace {

#ifndef SYRINGE_FILLER_WIFI_SSID
#define SYRINGE_FILLER_WIFI_SSID "SyringeFiller"
#endif
#ifndef SYRINGE_FILLER_WIFI_PASS
#define SYRINGE_FILLER_WIFI_PASS "syringe1234"
#endif

WebServer server(80);

constexpr size_t kMaxRecipeList = 64;
constexpr uint32_t kWifiConnectTimeoutMs = 20000;
constexpr uint32_t kWifiPollIntervalMs = 500;
constexpr char kMdnsHostname[] = "syringe-filler";

const char kIndexHtml[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Syringe Filler Recipes</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 24px; background: #f7f7f7; }
    h1 { margin: 0 0 12px; }
    .panel { background: white; padding: 16px; border-radius: 8px; box-shadow: 0 1px 3px rgba(0,0,0,0.12); }
    .row { display: flex; gap: 16px; flex-wrap: wrap; }
    .col { flex: 1 1 260px; }
    ul { list-style: none; padding: 0; margin: 0; }
    li { padding: 6px 8px; border-bottom: 1px solid #eee; cursor: pointer; }
    li:hover { background: #f0f0f0; }
    table { width: 100%; border-collapse: collapse; margin-top: 8px; }
    th, td { border-bottom: 1px solid #eee; padding: 6px; text-align: left; }
    input[type="number"], input[type="text"] { width: 100%; padding: 6px; }
    button { margin: 4px 4px 4px 0; padding: 6px 10px; }
    .muted { color: #666; font-size: 0.9em; }
  </style>
</head>
<body>
  <h1>Syringe Filler Recipes</h1>
  <div class="row">
    <div class="col panel">
      <h3>Recipes</h3>
      <button id="refresh">Refresh</button>
      <ul id="recipeList"></ul>
    </div>
    <div class="col panel">
      <h3>Editor</h3>
      <label>Recipe ID (hex)</label>
      <input id="recipeId" type="text" placeholder="e.g. 1A2B3C4D" />
      <div class="muted">Volume uses mL. Base slot is 1-based.</div>
      <table>
        <thead>
          <tr><th>Volume (mL)</th><th>Base Slot</th><th></th></tr>
        </thead>
        <tbody id="steps"></tbody>
      </table>
      <button id="add">Add Step</button>
      <button id="save">Save</button>
      <button id="del">Delete</button>
      <div id="status" class="muted"></div>
    </div>
  </div>
  <script>
    const listEl = document.getElementById('recipeList');
    const stepsEl = document.getElementById('steps');
    const recipeIdEl = document.getElementById('recipeId');
    const statusEl = document.getElementById('status');

    function setStatus(msg, ok = true) {
      statusEl.textContent = msg;
      statusEl.style.color = ok ? '#2b6' : '#c33';
    }

    function addRow(step = { volume_ml: 1, base_slot: 1 }) {
      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td><input type="number" step="0.1" min="0.1" value="${step.volume_ml}" /></td>
        <td><input type="number" min="1" value="${step.base_slot}" /></td>
        <td><button class="remove">Remove</button></td>
      `;
      tr.querySelector('.remove').onclick = () => tr.remove();
      stepsEl.appendChild(tr);
    }

    async function refreshList() {
      const resp = await fetch('/api/recipes');
      const data = await resp.json();
      listEl.innerHTML = '';
      (data.recipes || []).forEach(recipeId => {
        const li = document.createElement('li');
        li.textContent = recipeId;
        li.onclick = () => loadRecipe(recipeId);
        listEl.appendChild(li);
      });
      setStatus('Loaded recipes.');
    }

    async function loadRecipe(recipeId) {
      recipeIdEl.value = recipeId;
      const resp = await fetch(`/api/recipes/${recipeId}`);
      if (!resp.ok) {
        setStatus('Recipe not found.', false);
        stepsEl.innerHTML = '';
        return;
      }
      const data = await resp.json();
      stepsEl.innerHTML = '';
      (data.steps || []).forEach(addRow);
      if ((data.steps || []).length === 0) addRow();
      setStatus('Recipe loaded.');
    }

    async function saveRecipe() {
      const recipeId = recipeIdEl.value.trim();
      if (!recipeId) return setStatus('Recipe ID is required.', false);
      const steps = Array.from(stepsEl.querySelectorAll('tr')).map(row => {
        const inputs = row.querySelectorAll('input');
        return {
          volume_ml: parseFloat(inputs[0].value || '0'),
          base_slot: parseInt(inputs[1].value || '0', 10)
        };
      });
      const resp = await fetch(`/api/recipes/${recipeId}`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ steps })
      });
      if (resp.ok) {
        setStatus('Recipe saved.');
        refreshList();
      } else {
        const msg = await resp.text();
        setStatus(`Save failed: ${msg}`, false);
      }
    }

    async function deleteRecipe() {
      const recipeId = recipeIdEl.value.trim();
      if (!recipeId) return setStatus('Recipe ID is required.', false);
      if (!confirm('Delete this recipe?')) return;
      const resp = await fetch(`/api/recipes/${recipeId}`, { method: 'DELETE' });
      if (resp.ok) {
        setStatus('Recipe deleted.');
        stepsEl.innerHTML = '';
        refreshList();
      } else {
        const msg = await resp.text();
        setStatus(`Delete failed: ${msg}`, false);
      }
    }

    document.getElementById('refresh').onclick = refreshList;
    document.getElementById('add').onclick = () => addRow();
    document.getElementById('save').onclick = saveRecipe;
    document.getElementById('del').onclick = deleteRecipe;

    refreshList();
  </script>
</body>
</html>
)HTML";

String recipeIdToHex(uint32_t recipeId) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%08X", recipeId);
  return String(buf);
}

bool parseRecipeId(const String& hexStr, uint32_t& out) {
  if (hexStr.length() == 0) return false;
  out = strtoul(hexStr.c_str(), nullptr, 16);
  return out != 0;
}

void sendJson(const JsonDocument& doc) {
  String body;
  serializeJson(doc, body);
  server.send(200, "application/json", body);
}

void handleListRecipes() {
  uint32_t ids[kMaxRecipeList];
  size_t count = 0;
  if (!Util::listRecipeIds(ids, kMaxRecipeList, count)) {
    server.send(500, "text/plain", "Failed to list recipes");
    return;
  }

  JsonDocument doc;
  JsonArray arr = doc["recipes"].to<JsonArray>();
  for (size_t i = 0; i < count; ++i) {
    arr.add(recipeIdToHex(ids[i]));
  }
  sendJson(doc);
}

void handleGetRecipe(uint32_t recipeId) {
  Util::Recipe recipe;
  if (!Util::loadRecipe(recipeId, recipe)) {
    server.send(404, "text/plain", "Recipe not found");
    return;
  }

  JsonDocument doc;
  doc["recipe_id"] = recipeIdToHex(recipeId);
  JsonArray arr = doc["steps"].to<JsonArray>();
  recipe.toJson(arr);
  sendJson(doc);
}

void handlePutRecipe(uint32_t recipeId) {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "Missing body");
    return;
  }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }

  JsonArrayConst arr;
  if (doc.is<JsonArray>()) {
    arr = doc.as<JsonArrayConst>();
  } else {
    arr = doc["steps"].as<JsonArrayConst>();
  }

  if (arr.isNull()) {
    server.send(400, "text/plain", "Missing steps array");
    return;
  }

  Util::Recipe recipe;
  recipe.fromJson(arr);
  if (recipe.isEmpty()) {
    server.send(400, "text/plain", "Recipe invalid or empty");
    return;
  }

  if (!Util::saveRecipe(recipeId, recipe)) {
    server.send(500, "text/plain", "Save failed");
    return;
  }

  server.send(200, "text/plain", "OK");
}

void handleDeleteRecipe(uint32_t recipeId) {
  if (!Util::deleteRecipe(recipeId)) {
    server.send(404, "text/plain", "Delete failed");
    return;
  }
  server.send(200, "text/plain", "OK");
}

void handleApiRecipes() {
  if (server.method() == HTTP_GET) {
    handleListRecipes();
    return;
  }
  server.send(405, "text/plain", "Method not allowed");
}

void handleApiRecipeItem() {
  String uri = server.uri();
  const String prefix = "/api/recipes/";
  if (!uri.startsWith(prefix)) {
    server.send(404, "text/plain", "Not found");
    return;
  }

  String hexStr = uri.substring(prefix.length());
  uint32_t recipeId = 0;
  if (!parseRecipeId(hexStr, recipeId)) {
    server.send(400, "text/plain", "Invalid recipe ID");
    return;
  }

  if (server.method() == HTTP_GET) {
    handleGetRecipe(recipeId);
  } else if (server.method() == HTTP_PUT) {
    handlePutRecipe(recipeId);
  } else if (server.method() == HTTP_DELETE) {
    handleDeleteRecipe(recipeId);
  } else {
    server.send(405, "text/plain", "Method not allowed");
  }
}

const char* wifiStatusToString(wl_status_t status) {
  switch (status) {
    case WL_NO_SSID_AVAIL: return "NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "SCAN_COMPLETED";
    case WL_CONNECTED: return "CONNECTED";
    case WL_CONNECT_FAILED: return "CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST";
    case WL_DISCONNECTED: return "DISCONNECTED";
    case WL_IDLE_STATUS: return "IDLE";
    default: return "UNKNOWN";
  }
}

bool startMdns() {
  if (!MDNS.begin(kMdnsHostname)) {
    Serial.println("[WebUI] mDNS failed to start.");
    return false;
  }
  MDNS.addService("http", "tcp", 80);
  Serial.printf("[WebUI] mDNS started: http://%s.local/\n", kMdnsHostname);
  return true;
}

bool connectToWiFi(const String& ssid, const String& password) {
  Serial.printf("[WebUI] Connecting to WiFi SSID '%s'...\n", ssid.c_str());
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long start = millis();
  wl_status_t lastStatus = WL_IDLE_STATUS;
  while (millis() - start < kWifiConnectTimeoutMs) {
    wl_status_t status = WiFi.status();
    if (status != lastStatus) {
      Serial.printf("[WebUI] WiFi status: %s\n", wifiStatusToString(status));
      lastStatus = status;
    }
    if (status == WL_CONNECTED) {
      IPAddress ip = WiFi.localIP();
      Serial.printf("[WebUI] WiFi connected. IP: %s RSSI: %ld dBm\n",
                    ip.toString().c_str(), WiFi.RSSI());
      startMdns();
      return true;
    }
    delay(kWifiPollIntervalMs);
  }

  Serial.println("[WebUI] WiFi connection timed out.");
  return false;
}

void startAccessPoint() {
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(SYRINGE_FILLER_WIFI_SSID, SYRINGE_FILLER_WIFI_PASS);
  if (!ok) {
    Serial.println("[WebUI] Failed to start WiFi AP.");
    return;
  }
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("[WebUI] AP '%s' started. IP: %s\n", SYRINGE_FILLER_WIFI_SSID, ip.toString().c_str());
  Serial.println("[WebUI] Open http://192.168.4.1/wifi to configure STA WiFi.");
}

void startWiFi() {
  String ssid;
  String password;
  if (Util::loadWiFiCredentials(ssid, password)) {
    if (connectToWiFi(ssid, password)) {
      return;
    }
    Serial.println("[WebUI] Falling back to AP mode.");
  } else {
    Serial.println("[WebUI] No saved WiFi credentials found.");
  }
  startAccessPoint();
}

} // namespace

namespace App {
namespace WebUI {

void begin() {
  startWiFi();

  server.on("/", HTTP_GET, []() {
    server.send_P(200, "text/html", kIndexHtml);
  });
  server.on("/index.html", HTTP_GET, []() {
    server.send_P(200, "text/html", kIndexHtml);
  });
  server.on("/api/recipes", HTTP_ANY, handleApiRecipes);
  server.onNotFound(handleApiRecipeItem);

  server.begin();
  Serial.println("[WebUI] HTTP server started on port 80.");
}

void handle() {
  server.handleClient();
}

} // namespace WebUI
} // namespace App
