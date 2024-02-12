# NOCTIS
NOCTIS contains all the control and GUI software intended to control a Parrot ANAFI USA drone. It's purpose to find and tracking target out of a defined area using the map editor.

# Setup
NOCTIS was developed on Ubuntu 22.04, however as long as your environment supports [Olympe](https://developer.parrot.com/docs/olympe/index.html) the software should work. NOTE: [Sphinx](https://developer.parrot.com/docs/sphinx/index.html) only supports Linux.

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

## API Key
To run the WebUI of NOCTIS you must add your API key for Google Maps otherwise the map editor will not function. I recommend adding a new file under `WebUI` and adding the following code snippet.
```C#
namespace WebUI.Env
{
  public class Vars
  {
    public const string API_KEY = "{Your Google Maps API Key}";
  }
}
```
## Run
To launch the GUI first navigate to `WebUI` and start the `dotnet` project.
```bash
cd ./WebUI
dotnet run
```

To use Sphinx as a simulated drone execute `start_sphinx.sh`.
```bash
./start_sphinx.sh
```
