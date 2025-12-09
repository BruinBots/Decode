package org.firstinspires.ftc.teamcode.Refactor.Utils;

import fi.iki.elonen.NanoHTTPD;
import java.io.*;

public class FileDownloadServer extends NanoHTTPD {

    public FileDownloadServer(int port) throws IOException {
        super(port);
        start(SOCKET_READ_TIMEOUT, false);
        System.out.println("FileDownloadServer started on port " + port);
    }

    @Override
    public Response serve(IHTTPSession session) {
        if (session.getUri().equals("/download")) {
            String filePath = session.getParms().get("path");
            if (filePath == null) {
                return newFixedLengthResponse(Response.Status.BAD_REQUEST,
                        "text/plain", "Missing ?path=");
            }

            File f = new File(filePath);
            if (!f.exists()) {
                return newFixedLengthResponse(Response.Status.NOT_FOUND,
                        "text/plain", "File not found");
            }

            try {
                FileInputStream fis = new FileInputStream(f);
                return newChunkedResponse(Response.Status.OK,
                        "application/octet-stream", fis);
            } catch (Exception e) {
                return newFixedLengthResponse(Response.Status.INTERNAL_ERROR,
                        "text/plain", e.toString());
            }
        }

        return newFixedLengthResponse("OK – Server is running");
    }
}