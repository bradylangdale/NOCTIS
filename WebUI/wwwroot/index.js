var polygons = [];
var geofence = [];

// TODO: replace the hard coded lat and long with something more customizable
function initGeofenceEditor() {
    const map = new google.maps.Map(document.getElementById("map"), {
        center: { lat: 30.70439747044987, lng: -96.32200039372728 },
        zoom: 17,
        mapTypeId: "satellite",
    });

    const drawingManager = new google.maps.drawing.DrawingManager({
        drawingMode: null,
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

    for (let i = 0; i < polygons.length; i++)
    {
        polygons[i].setMap(map);

        google.maps.event.addListener(polygons[i].getPath(), 'insert_at', function() {
            console.log("polygon insert_at triggered");
         });

        google.maps.event.addListener(polygons[i].getPath(), 'remove_at', function() {
            console.log("polygon remove_at triggered");
        });

        google.maps.event.addListener(polygons[i].getPath(), 'set_at', function() {
            console.log("polygon set_at triggered");
        });

        google.maps.event.addListener(polygons[i], "contextmenu", (e) => {
            // Check if click was on a vertex control point
            if (e.vertex == undefined) {
                return;
            }

            console.log("polygon vertex right clicked");
            deleteMenu.open(map, polygons[i].getPath(), e.vertex);
        });

        polygons[i].addListener("click", menuFunc);
    }

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
                geofence[polygons.indexOf(polygon)].type = 'KIA';
                polygon.setOptions({fillColor: "#34A853", strokeColor:"#34A853"});
            });

            google.maps.event.addDomListener(document.getElementById("KOA"), 'click', function(e) {
                console.log("Set as Keep Out Area");
                geofence[polygons.indexOf(polygon)].type = 'KOA';
                polygon.setOptions({fillColor: "#ff2617", strokeColor:"#ff2617"});
            });

            google.maps.event.addDomListener(document.getElementById("EA"), 'click', function(e) {
                console.log("Set as Exit Area");
                geofence[polygons.indexOf(polygon)].type = 'EA';
                polygon.setOptions({fillColor: "#fbff00", strokeColor:"#fbff00"});
            });

            google.maps.event.addDomListener(document.getElementById("DP"), 'click', function(e) {
                console.log("Deleting Polygon");
                var i = polygons.indexOf(polygon);
                geofence.pop(i);
                polygons.pop(i);
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
        polygons.push(polygon);
        geofence.push({type: 'n/a', shape: polygon});

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

window.geofenceEditor = {
    getData: function()
    {
        var fence = [];

        for (let i = 0; i < geofence.length; i++)
        {
            const vertices = geofence[i].shape.getPath();
            var coords = [];
            for (let j = 0; j < vertices.getLength(); j++)
            {
                coords.push([vertices.getAt(j).lat(), vertices.getAt(j).lng()]);
            }
            fence.push({type: geofence[i].type, data: coords});
        }

        return fence;
    },

    setData: function(data)
    {
        polygons = [];
        geofence = [];
        
        const obj = JSON.parse(data);

        for (let i = 0; i < obj.length; i++)
        {
            var coords = [];

            for (let j = 0; j < obj[i].data.length; j++)
            {
                coords.push({ lat: obj[i].data[j][0], lng: obj[i].data[j][1] });
            }

            var options = null;
            switch (obj[i].type)
            {
                case 'KIA':
                    options = {
                        paths: coords,
                        fillColor: "#34A853",
                        fillOpacity: 0.3,
                        strokeColor:"#34A853",
                        strokeWeight: 5,
                        clickable: true,
                        editable: true,
                        zIndex: 1,
                    };
                    break;
                case 'KOA':
                    options = {
                        paths: coords,
                        fillColor: "#ff2617",
                        fillOpacity: 0.3,
                        strokeColor:"#ff2617",
                        strokeWeight: 5,
                        clickable: true,
                        editable: true,
                        zIndex: 1,
                    };
                    break;
                case 'EA':
                    options = {
                        paths: coords,
                        fillColor: "#fbff00",
                        fillOpacity: 0.3,
                        strokeColor:"#fbff00",
                        strokeWeight: 5,
                        clickable: true,
                        editable: true,
                        zIndex: 1,
                    };
                    break;
                default:
                    console.log("Unknown type:" + obj[i].type);
            }
            
            var poly = new google.maps.Polygon(options);
            polygons.push(
                poly
            );

            geofence.push({type: obj[i].type, shape: poly});
        }
    },
};