#ifndef ANCHOR_LOGIC_H
#define ANCHOR_LOGIC_H

#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

#ifndef radians
#define radians(deg) ((deg) * PI / 180.0)
#endif

// Haversine distance between two GPS coordinates in meters
inline double haversineDistance(double lat1, double lng1, double lat2, double lng2) {
  const double R = 6371000.0;
  double dLat = radians(lat2 - lat1);
  double dLng = radians(lng2 - lng1);
  double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
             cos(radians(lat1)) * cos(radians(lat2)) *
             sin(dLng / 2.0) * sin(dLng / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return R * c;
}

// Anchor watch state
struct AnchorState {
  double anchorLat;
  double anchorLng;
  bool anchorSet;
  double alarmRadius;
  double currentDistance;
  bool alarmActive;
  bool alarmSilenced;

  AnchorState() : anchorLat(0), anchorLng(0), anchorSet(false),
                  alarmRadius(30.0), currentDistance(0), 
                  alarmActive(false), alarmSilenced(false) {}

  void setAnchor(double lat, double lng) {
    anchorLat = lat;
    anchorLng = lng;
    anchorSet = true;
    alarmActive = false;
    alarmSilenced = false;
    currentDistance = 0.0;
  }

  // Returns true if alarm state changed
  bool updatePosition(double lat, double lng) {
    if (!anchorSet) return false;

    currentDistance = haversineDistance(anchorLat, anchorLng, lat, lng);
    bool wasActive = alarmActive;

    if (currentDistance > alarmRadius) {
      alarmActive = true;
    } else {
      alarmActive = false;
      alarmSilenced = false;
    }

    return alarmActive != wasActive;
  }

  void silenceAlarm() {
    alarmSilenced = true;
  }

  // Turn the anchor watch off entirely
  void disarm() {
    anchorSet = false;
    alarmActive = false;
    alarmSilenced = false;
    currentDistance = 0.0;
  }

  bool shouldBuzzerSound() const {
    return alarmActive && !alarmSilenced;
  }

  bool setRadius(double r) {
    if (r >= 5.0 && r <= 500.0) {
      alarmRadius = r;
      alarmSilenced = false;
      return true;
    }
    return false;
  }
};

#endif // ANCHOR_LOGIC_H
