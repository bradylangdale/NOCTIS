# NOCTIS
NOCTIS contains all the control and GUI software intended to control a Parrot ANAFI USA drone. It's purpose to find and tracking target out of a defined area using the map editor.

# Setup
NOCTIS was developed on Ubuntu 22.04, however as long as your environment supports [Olympe](https://developer.parrot.com/docs/olympe/index.html) the software should work.

NOCTIS also supports the Raspberry Pi 4B as this was the primary target for software. I will later include build instructions for compiling Olympe from source on the RPi but TBA for now.

NOTE: [Sphinx](https://developer.parrot.com/docs/sphinx/index.html) only supports Linux.

## Dotnet
You will need at least Dotnet 7.0. The following commands should work for Ubuntu 22.04.
```bash
sudo apt-get update && \
  sudo apt-get install -y dotnet-sdk-7.0

sudo apt-get update && \
  sudo apt-get install -y aspnetcore-runtime-7.0
```

## Python
You will need the following Python packages. Sphinx is necessary to run everything locally in a simulation however this is not required.
```bash
pip install parrot-olympe

curl --fail --silent --show-error --location https://debian.parrot.com/gpg | gpg --dearmor | sudo tee /usr/share/keyrings/debian.parrot.com.gpg > /dev/null
echo "deb [signed-by=/usr/share/keyrings/debian.parrot.com.gpg] https://debian.parrot.com/ $(lsb_release -cs) main generic" | sudo tee /etc/apt/sources.list.d/debian.parrot.com.list
sudo apt update

sudo apt install parrot-sphinx
sudo apt install parrot-ue4-empty
```
For Google Maps you'll need to install the planet map.
```bash
sudo apt install parrot-ue4-planet
```

## API Keys
To run the WebUI of NOCTIS you must add your API key for Google Maps otherwise the map editor will not function. I have include 'environment.cs.example' for you use and replace with your own API key.
```C#
namespace WebUI.Env
{
  public class Vars
  {
    public const string API_KEY = "{YOUR_GOOGLE_API_KEY}";
  }
}
```
Additionally, if you'd like to use Google Maps with Parrot Sphinx for simulation at specific coordinates be sure to add you keys to 'planet.yaml.example' and save it as 'planet.yaml'.
```yaml
Cesium:
  - GoogleKey: '{YOUR_GOOGLE_API_KEY}'
    CesiumToken: '{YOUR_CESIUM_API_KEY}'
    TilesetSource: 'Google'
    IonAssetId: 2275207
```

## Run
To launch the GUI first navigate to `WebUI` and start the `dotnet` project.
```bash
cd ./WebUI
dotnet run
```

To use Sphinx as a simulated drone execute `start_example.sh` or with Google Maps try `./start_pic.sh`.
```bash
./start_pic.sh
```
