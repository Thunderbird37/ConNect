#pragma once

#include <cstddef>
#include <string>

// Hardwareunabhaengige Helfer fuer das OTA-Manifest (host-testbar).
namespace ota {

inline bool startsWith(const std::string& value, const char* prefix) {
    return value.rfind(prefix, 0) == 0;
}

inline size_t skipJsonWhitespace(const std::string& text, size_t pos) {
    while (pos < text.size() &&
           (text[pos] == ' ' || text[pos] == '\t' || text[pos] == '\r' || text[pos] == '\n')) {
        ++pos;
    }
    return pos;
}

// Liefert den rohen (nicht entescapten) String-Wert eines JSON-Schluessels
// oder "", wenn der Schluessel fehlt oder keinen String-Wert hat.
inline std::string extractJsonStringField(const std::string& payload, const char* field) {
    const std::string needle = std::string("\"") + field + "\"";
    size_t searchFrom = 0;
    while (true) {
        size_t fieldIdx = payload.find(needle, searchFrom);
        if (fieldIdx == std::string::npos) return "";
        searchFrom = fieldIdx + 1;

        size_t pos = skipJsonWhitespace(payload, fieldIdx + needle.size());
        // Treffer ohne folgenden ':' ist ein String-Wert, kein Schluessel.
        if (pos >= payload.size() || payload[pos] != ':') continue;
        pos = skipJsonWhitespace(payload, pos + 1);
        if (pos >= payload.size() || payload[pos] != '"') return "";

        const size_t valueStart = pos + 1;
        bool escaped = false;
        for (size_t i = valueStart; i < payload.size(); ++i) {
            const char current = payload[i];
            if (escaped) {
                escaped = false;
            } else if (current == '\\') {
                escaped = true;
            } else if (current == '"') {
                return payload.substr(valueStart, i - valueStart);
            }
        }
        return "";
    }
}

inline bool isValidMD5(const std::string& value) {
    if (value.size() != 32) return false;
    for (char c : value) {
        const bool isHex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
        if (!isHex) return false;
    }
    return true;
}

inline std::string buildAbsoluteUrl(const std::string& sourceUrl, const std::string& location) {
    if (location.empty()) return "";
    if (startsWith(location, "http://") || startsWith(location, "https://")) return location;

    const size_t schemeIdx = sourceUrl.find("://");
    if (schemeIdx == std::string::npos) return location;
    if (startsWith(location, "//")) return sourceUrl.substr(0, schemeIdx + 1) + location;

    const size_t hostStart = schemeIdx + 3;
    const std::string base = sourceUrl.substr(0, sourceUrl.find_first_of("?#", hostStart));
    const size_t hostEnd = base.find('/', hostStart);
    const std::string origin = hostEnd != std::string::npos ? base.substr(0, hostEnd) : base;

    if (location[0] == '/') return origin + location;
    if (hostEnd == std::string::npos) return origin + "/" + location;
    return base.substr(0, base.rfind('/') + 1) + location;
}

enum class UpdateDecision {
    Install,
    UpToDate,
    RemoteOlder,
};

inline std::string stripVersionPrefix(const std::string& version) {
    if (!version.empty() && (version[0] == 'v' || version[0] == 'V')) return version.substr(1);
    return version;
}

// Parst exakt MAJOR.MINOR.PATCH.
inline bool parseSemver(const std::string& text, unsigned long (&parts)[3]) {
    size_t pos = 0;
    for (int index = 0; index < 3; ++index) {
        if (pos >= text.size() || text[pos] < '0' || text[pos] > '9') return false;
        unsigned long value = 0;
        while (pos < text.size() && text[pos] >= '0' && text[pos] <= '9') {
            value = value * 10 + static_cast<unsigned long>(text[pos] - '0');
            if (value > 1000000UL) return false;
            ++pos;
        }
        parts[index] = value;
        if (index < 2) {
            if (pos >= text.size() || text[pos] != '.') return false;
            ++pos;
        }
    }
    return pos == text.size();
}

// currentVersion: git-describe-Format TAG[-COMMITS-gHASH][-dirty], "dev" o.ae.
// Dirty-, Dev- und reine Hash-Builds duerfen immer aktualisiert werden.
inline UpdateDecision decideUpdate(const std::string& currentVersion, const std::string& remoteVersion) {
    const std::string remote = stripVersionPrefix(remoteVersion);
    const std::string local = stripVersionPrefix(currentVersion);
    const std::string localBase = local.substr(0, local.find('-'));

    const bool isDirty = currentVersion.find("dirty") != std::string::npos;
    const bool isDevBuild = currentVersion == "dev" || currentVersion == "unknown" || currentVersion.size() < 3;
    const bool isHashOnly = localBase.find('.') == std::string::npos;
    if (isDirty || isDevBuild || isHashOnly) return UpdateDecision::Install;

    unsigned long remoteParts[3] = {};
    unsigned long localParts[3] = {};
    if (parseSemver(remote, remoteParts) && parseSemver(localBase, localParts)) {
        for (int index = 0; index < 3; ++index) {
            if (remoteParts[index] != localParts[index]) {
                return remoteParts[index] > localParts[index] ? UpdateDecision::Install
                                                               : UpdateDecision::RemoteOlder;
            }
        }
        return UpdateDecision::UpToDate;
    }
    return remote == localBase ? UpdateDecision::UpToDate : UpdateDecision::Install;
}

} // namespace ota
