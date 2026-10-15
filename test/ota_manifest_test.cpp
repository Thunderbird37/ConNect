#include "ota_manifest.h"

#include <cassert>
#include <string>

using ota::UpdateDecision;

static void testExtractsGitHubManifest() {
    const std::string manifest = R"({
  "version": "v2.0.1",
  "firmware": "https://github.com/Thunderbird37/ConNect/releases/download/v2.0.1/firmware.bin",
  "md5": "f1e4cc41faacf3eb7eeeee8c6ec3acdf"
})";
    assert(ota::extractJsonStringField(manifest, "version") == "v2.0.1");
    assert(ota::extractJsonStringField(manifest, "firmware") ==
           "https://github.com/Thunderbird37/ConNect/releases/download/v2.0.1/firmware.bin");
    assert(ota::extractJsonStringField(manifest, "md5") == "f1e4cc41faacf3eb7eeeee8c6ec3acdf");
    assert(ota::extractJsonStringField(manifest, "tag").empty());
}

static void testJsonKeyMustBeAKey() {
    // "version" als Wert darf nicht als Schluessel erkannt werden.
    const std::string manifest = R"({"notes":"version","version":"v1.2.3"})";
    assert(ota::extractJsonStringField(manifest, "version") == "v1.2.3");
}

static void testJsonNonStringValueIsIgnored() {
    // Frueher wurde hier der naechste String ("firmware") als Version gelesen.
    const std::string manifest = R"({"version": 2, "firmware":"fw.bin"})";
    assert(ota::extractJsonStringField(manifest, "version").empty());
    assert(ota::extractJsonStringField(manifest, "firmware") == "fw.bin");
}

static void testJsonEscapedQuotes() {
    const std::string manifest = R"({"notes":"say \"hi\"","url":"a\\b"})";
    assert(ota::extractJsonStringField(manifest, "notes") == R"(say \"hi\")");
    assert(ota::extractJsonStringField(manifest, "url") == R"(a\\b)");
    assert(ota::extractJsonStringField(R"({"version":"unterminated)", "version").empty());
    assert(ota::extractJsonStringField(R"({"version":""})", "version").empty());
}

static void testMd5Validation() {
    assert(ota::isValidMD5("f1e4cc41faacf3eb7eeeee8c6ec3acdf"));
    assert(ota::isValidMD5("F1E4CC41FAACF3EB7EEEEE8C6EC3ACDF"));
    assert(!ota::isValidMD5("f1e4cc41faacf3eb7eeeee8c6ec3acd"));
    assert(!ota::isValidMD5("g1e4cc41faacf3eb7eeeee8c6ec3acdf"));
    assert(!ota::isValidMD5(""));
}

static void testAbsoluteUrls() {
    const std::string manifest = "https://example.com/ota/latest.json";
    assert(ota::buildAbsoluteUrl(manifest, "").empty());
    assert(ota::buildAbsoluteUrl(manifest, "http://other.net/fw.bin") == "http://other.net/fw.bin");
    assert(ota::buildAbsoluteUrl(manifest, "/files/fw.bin") == "https://example.com/files/fw.bin");
    assert(ota::buildAbsoluteUrl(manifest, "fw.bin") == "https://example.com/ota/fw.bin");
    assert(ota::buildAbsoluteUrl(manifest, "//cdn.example.com/fw.bin") == "https://cdn.example.com/fw.bin");
    assert(ota::buildAbsoluteUrl("https://example.com", "fw.bin") == "https://example.com/fw.bin");
    assert(ota::buildAbsoluteUrl("https://example.com/ota/latest.json?token=a/b", "fw.bin") ==
           "https://example.com/ota/fw.bin");
    assert(ota::buildAbsoluteUrl("latest.json", "fw.bin") == "fw.bin");
}

static void testUpdateDecisions() {
    assert(ota::decideUpdate("v2.0.1", "v2.0.2") == UpdateDecision::Install);
    assert(ota::decideUpdate("v2.0.1", "v2.1.0") == UpdateDecision::Install);
    assert(ota::decideUpdate("v2.0.1", "v10.0.0") == UpdateDecision::Install);
    assert(ota::decideUpdate("v2.0.1", "v2.0.1") == UpdateDecision::UpToDate);
    assert(ota::decideUpdate("2.0.1", "V2.0.1") == UpdateDecision::UpToDate);
    assert(ota::decideUpdate("v2.0.1", "v2.0.0") == UpdateDecision::RemoteOlder);
    assert(ota::decideUpdate("v2.0.10", "v2.0.9") == UpdateDecision::RemoteOlder);

    // Commits nach dem Tag zaehlen als dieselbe Basisversion.
    assert(ota::decideUpdate("v2.0.1-3-gabc1234", "v2.0.1") == UpdateDecision::UpToDate);
    assert(ota::decideUpdate("v2.0.1-3-gabc1234", "v2.0.2") == UpdateDecision::Install);

    // Dirty-, Dev- und Hash-Builds duerfen immer aktualisiert werden.
    assert(ota::decideUpdate("v2.0.1-dirty", "v2.0.1") == UpdateDecision::Install);
    assert(ota::decideUpdate("v2.0.1-3-gabc1234-dirty", "v1.0.0") == UpdateDecision::Install);
    assert(ota::decideUpdate("dev", "v1.0.0") == UpdateDecision::Install);
    assert(ota::decideUpdate("unknown", "v1.0.0") == UpdateDecision::Install);
    assert(ota::decideUpdate("7c637d3", "v1.0.0") == UpdateDecision::Install);

    // Nicht-semantische Versionen: nur exakte Gleichheit gilt als aktuell.
    assert(ota::decideUpdate("v2.0.1", "firmware.bin") == UpdateDecision::Install);
    assert(ota::decideUpdate("v2.0", "v2.0") == UpdateDecision::UpToDate);
    assert(ota::decideUpdate("v2.0", "v2.1") == UpdateDecision::Install);
}

int main() {
    testExtractsGitHubManifest();
    testJsonKeyMustBeAKey();
    testJsonNonStringValueIsIgnored();
    testJsonEscapedQuotes();
    testMd5Validation();
    testAbsoluteUrls();
    testUpdateDecisions();
    return 0;
}
