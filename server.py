import http.server
import socketserver
import mimetypes

PORT = 8000

class CustomHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        # Fix MIME types for .js and .wasm
        if self.path.endswith(".js"):
            self.send_header("Content-type", "application/javascript")
        elif self.path.endswith(".wasm"):
            self.send_header("Content-type", "application/wasm")
        else:
            super().end_headers()
            return
        super().end_headers()

    def guess_type(self, path):
        if path.endswith(".js"):
            return "application/javascript"
        elif path.endswith(".wasm"):
            return "application/wasm"
        else:
            return super().guess_type(path)

with socketserver.TCPServer(("", PORT), CustomHandler) as httpd:
    print(f"Serving at port {PORT}")
    httpd.serve_forever()
