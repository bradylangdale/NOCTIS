// TODO: replace the hard coded lat and long with something more customizable
function initGeofenceEditor() {
    const map = new google.maps.Map(document.getElementById("map"), {
      center: { lat: 30.70439747044987, lng: -96.32200039372728 },
      zoom: 17,
      mapTypeId: "satellite",
    });
    const drawingManager = new google.maps.drawing.DrawingManager({
      drawingMode: google.maps.drawing.OverlayType.MARKER,
      drawingControl: true,
      drawingControlOptions: {
        position: google.maps.ControlPosition.TOP_CENTER,
        drawingModes: [
          google.maps.drawing.OverlayType.POLYGON,
        ],
      },
      markerOptions: {
        icon: "icons/anafi-usa.png",
      },
      polygonOptions: {
        fillColor: "#bababa",
        fillOpacity: 0.3,
        strokeColor:"#bababa",
        strokeWeight: 5,
        clickable: true,
        editable: true,
        zIndex: 1,
      },
    });
  
    drawingManager.setMap(map);

      /**
     * A menu that lets a user delete a selected vertex of a path.
     */
    class DeleteMenu extends google.maps.OverlayView {
      div_;
      divListener_;
      constructor() {
        super();
        this.div_ = document.createElement("div");
        this.div_.className = "delete-menu";
        this.div_.innerHTML = "Delete";

        const menu = this;

        google.maps.event.addDomListener(this.div_, "click", () => {
          menu.removeVertex();
        });
      }
      onAdd() {
        const deleteMenu = this;
        const map = this.getMap();

        this.getPanes().floatPane.appendChild(this.div_);
        // mousedown anywhere on the map except on the menu div will close the
        // menu.
        this.divListener_ = google.maps.event.addDomListener(
          map.getDiv(),
          "mousedown",
          (e) => {
            if (e.target != deleteMenu.div_) {
              deleteMenu.close();
            }
          },
          true,
        );
      }
      onRemove() {
        if (this.divListener_) {
          google.maps.event.removeListener(this.divListener_);
        }

        this.div_.parentNode.removeChild(this.div_);
        // clean up
        this.set("position", null);
        this.set("path", null);
        this.set("vertex", null);
      }
      close() {
        this.setMap(null);
      }
      draw() {
        const position = this.get("position");
        const projection = this.getProjection();

        if (!position || !projection) {
          return;
        }

        const point = projection.fromLatLngToDivPixel(position);

        this.div_.style.top = point.y + "px";
        this.div_.style.left = point.x + "px";
      }
      /**
       * Opens the menu at a vertex of a given path.
       */
      open(map, path, vertex) {
        this.set("position", path.getAt(vertex));
        this.set("path", path);
        this.set("vertex", vertex);
        this.setMap(map);
        this.draw();
      }
      /**
       * Deletes the vertex from the path.
       */
      removeVertex() {
        const path = this.get("path");
        const vertex = this.get("vertex");

        if (!path || vertex == undefined) {
          this.close();
          return;
        }

        path.removeAt(vertex);
        this.close();
      }
    }

    var infoWindow = new google.maps.InfoWindow();

    function menuFunc(event) {
      // Since this polygon has only one path, we can call getPath() to return the
      // MVCArray of LatLngs.
      // @ts-ignore
      infoWindow.close();
      infoWindow = new google.maps.InfoWindow();
      const polygon = this;
      const vertices = polygon.getPath();

      let contentString =
        '<b>Set Geofence Polygon Type</b><br>' +
        '<button id="KIA">Keep In Area</button><br>' +
        '<button id="KOA">Keep Out Area</button><br>' +
        '<button id="EA">Exit Area</button><br>' +
        '<button id="DP">Delete Polygon</button><br>'

      google.maps.event.addListener(infoWindow, 'domready', function() {
        google.maps.event.addDomListener(document.getElementById("KIA"), 'click', function(e) {
          console.log("Set as Keep In Area");
          polygon.setOptions({fillColor: "#34A853", strokeColor:"#34A853"});
        });
        google.maps.event.addDomListener(document.getElementById("KOA"), 'click', function(e) {
          console.log("Set as Keep Out Area");
          polygon.setOptions({fillColor: "#ff2617", strokeColor:"#ff2617"});
        });
        google.maps.event.addDomListener(document.getElementById("EA"), 'click', function(e) {
          console.log("Set as Exit Area");
          polygon.setOptions({fillColor: "#fbff00", strokeColor:"#fbff00"});
        });
        google.maps.event.addDomListener(document.getElementById("DP"), 'click', function(e) {
          console.log("Deleting Polygon");
          polygon.setMap(null);
        });
      });

      // Replace the info window's content and position.
      infoWindow.setContent(contentString);
      infoWindow.setPosition(event.latLng);
      infoWindow.open(map, this);
    }

    const deleteMenu = new DeleteMenu();

    google.maps.event.addListener(drawingManager, 'polygoncomplete', function(polygon) {
      google.maps.event.addListener(polygon.getPath(), 'insert_at', function() {
          console.log("polygon insert_at triggered");
      });

      google.maps.event.addListener(polygon.getPath(), 'remove_at', function() {
          console.log("polygon remove_at triggered");
      });

      google.maps.event.addListener(polygon.getPath(), 'set_at', function() {
          console.log("polygon set_at triggered");
      });

      google.maps.event.addListener(polygon, "contextmenu", (e) => {
        // Check if click was on a vertex control point
        if (e.vertex == undefined) {
          return;
        }
        
        console.log("polygon vertex right clicked");
        deleteMenu.open(map, polygon.getPath(), e.vertex);
      });

      polygon.addListener("click", menuFunc);
    });
  }

window.initGeofenceEditor = initGeofenceEditor