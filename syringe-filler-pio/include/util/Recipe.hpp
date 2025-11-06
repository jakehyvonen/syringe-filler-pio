#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

namespace Util {

// Represents one paint addition step (ordered)
struct RecipeEntry {
    float      volumeMl = 0.0f;     // always required
    int8_t     baseSlot = -1;       // 1-based index, -1 if none
    uint32_t   baseRFID = 0;        // optional
    String     colorHex;             // optional (e.g. "#12ABEF")
    uint32_t   paintId = 0;          // optional numeric ID

    // --- Validation ---
    bool isValid() const {
        return (volumeMl > 0.0f) && (
            baseSlot > 0 ||
            baseRFID != 0 ||
            (colorHex.length() > 0) ||
            paintId != 0
        );
    }

    // --- JSON (de)serialization ---
    void toJson(JsonObject obj) const {
        obj["volume_ml"] = volumeMl;
        if (baseSlot > 0) obj["base_slot"] = baseSlot;
        if (baseRFID != 0) obj["base_rfid"] = baseRFID;
        if (colorHex.length()) obj["color_hex"] = colorHex;
        if (paintId != 0) obj["paint_id"] = paintId;
    }

    void fromJson(const JsonObjectConst& obj) {
        volumeMl = obj["volume_ml"] | 0.0f;
        baseSlot = obj["base_slot"] | -1;
        baseRFID = obj["base_rfid"] | 0u;
        colorHex = obj["color_hex"] | "";
        paintId  = obj["paint_id"] | 0u;
    }
};

// A list of ordered recipe steps
struct Recipe {
    static constexpr uint8_t MAX_STEPS = 32;
    RecipeEntry steps[MAX_STEPS];
    uint8_t count = 0;

    void clear() { count = 0; }

    bool add(const RecipeEntry& e) {
        if (!e.isValid() || count >= MAX_STEPS) return false;
        steps[count++] = e;
        return true;
    }

    bool isEmpty() const { return count == 0; }

    // --- JSON I/O ---
    void toJson(JsonArray arr) const {
        for (uint8_t i = 0; i < count; ++i) {
            JsonObject o = arr.createNestedObject();
            steps[i].toJson(o);
        }
    }

    void fromJson(const JsonArrayConst& arr) {
        clear();
        for (JsonObjectConst o : arr) {
            RecipeEntry e;
            e.fromJson(o);
            if (e.isValid()) add(e);
        }
    }

    // --- Debug printing ---
    void printTo(Stream& s) const {
        s.printf("[Recipe] %u steps:\n", count);
        for (uint8_t i = 0; i < count; ++i) {
            const auto& e = steps[i];
            s.printf("  #%02u: %.2f mL | base=%d | RFID=%08lX | color=%s | paintId=%lu\n",
                     i+1, e.volumeMl, e.baseSlot,
                     e.baseRFID,
                     e.colorHex.length() ? e.colorHex.c_str() : "-",
                     e.paintId);
        }
    }
};

} // namespace Util
