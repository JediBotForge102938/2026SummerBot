package org.firstinspires.ftc.teamcode.telemetry;

final class TelemetryMarkup {
    private TelemetryMarkup() { }

    static String render(String html) {
        return html.replaceAll("<[^>]+>", "")
                .replace("&nbsp;", " ")
                .replace("&lt;", "<")
                .replace("&gt;", ">")
                .replace("&quot;", "\"")
                .replace("&#39;", "'")
                .replace("&amp;", "&");
    }
}
