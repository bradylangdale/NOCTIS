// This example requires the Drawing library. Include the libraries=drawing
// parameter when you first load the API. For example:
// <script src="https://maps.googleapis.com/maps/api/js?key=YOUR_API_KEY&libraries=drawing">
function initMap() {
    const map = new google.maps.Map(document.getElementById("map"), {
      center: { lat: 30.70439747044987, lng: -96.32200039372728 },
      zoom: 17,
    });
    const drawingManager = new google.maps.drawing.DrawingManager({
      drawingMode: google.maps.drawing.OverlayType.MARKER,
      drawingControl: true,
      drawingControlOptions: {
        position: google.maps.ControlPosition.TOP_CENTER,
        drawingModes: [
          google.maps.drawing.OverlayType.MARKER,
          google.maps.drawing.OverlayType.POLYGON,
          google.maps.drawing.OverlayType.RECTANGLE,
        ],
      },
      markerOptions: {
        icon: "https://developers.google.com/maps/documentation/javascript/examples/full/images/beachflag.png",
      },
      polygonOptions: {
        fillColor: "#34A853",
        fillOpacity: 0.25,
        strokeColor:"#34A853",
        strokeWeight: 5,
        clickable: false,
        editable: true,
        zIndex: 1,
      },
    });
  
    drawingManager.setMap(map);
  }
  
  window.initMap = initMap;
