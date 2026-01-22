import { app, BrowserWindow, dialog, session } from "electron";
import path from "path";
import { fileURLToPath } from "url";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

app.commandLine.appendSwitch("enable-experimental-web-platform-features");
app.commandLine.appendSwitch("enable-features", "Serial");

function createWindow() {
  const win = new BrowserWindow({
    width: 1000,
    height: 900,
    webPreferences: {
      contextIsolation: false,
      nodeIntegration: true,
    },
  });
  win.loadFile(path.join(__dirname, "index.html"));
}

app.whenReady().then(() => {
  session.defaultSession.setPermissionRequestHandler((webContents, permission, callback) => {
    if (permission === "serial") {
      callback(true);
      return;
    }
    callback(false);
  });

  session.defaultSession.setPermissionCheckHandler((webContents, permission) => {
    if (permission === "serial") return true;
    return false;
  });

  session.defaultSession.on("select-serial-port", async (event, portList, webContents, callback) => {
    event.preventDefault();
    if (!portList || portList.length === 0) {
      callback("");
      return;
    }
    const preferred = portList.find((port) => port.path === "/dev/ttyUSB1");
    if (preferred) {
      callback(preferred.portId);
      return;
    }
    if (portList.length === 1) {
      callback(portList[0].portId);
      return;
    }
    const list = portList.map((port) => {
      const path = port.path || port.displayName || "unknown";
      const man = port.manufacturer ? " (" + port.manufacturer + ")" : "";
      return "- " + path + man;
    }).join("\n");
    const result = await dialog.showMessageBox({
      type: "question",
      buttons: ["Use First Port", "Cancel"],
      defaultId: 0,
      cancelId: 1,
      message: "Multiple serial ports detected",
      detail: "Available ports:\n" + list,
    });
    if (result.response === 0) {
      callback(portList[0].portId);
    } else {
      callback("");
    }
  });

  createWindow();
});

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") app.quit();
});
