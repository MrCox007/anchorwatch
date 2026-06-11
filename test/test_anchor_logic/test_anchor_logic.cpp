#include <unity.h>
#include "anchor_logic.h"
#include <math.h>

// ======== Haversine Distance Tests ========

void test_same_point_distance_is_zero(void) {
    double d = haversineDistance(59.0, 10.0, 59.0, 10.0);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, d);
}

void test_known_distance_oslo_to_copenhagen(void) {
    // Oslo (59.9139, 10.7522) to Copenhagen (55.6761, 12.5683) ≈ 482 km
    double d = haversineDistance(59.9139, 10.7522, 55.6761, 12.5683);
    TEST_ASSERT_DOUBLE_WITHIN(5000.0, 482000.0, d); // within 5km
}

void test_short_distance_30m(void) {
    // ~30m offset at lat 59: moving ~0.00027 degrees in latitude
    double lat = 59.0;
    double lng = 10.0;
    double offset = 30.0 / 111320.0; // approx degrees per meter at equator adjusted
    double d = haversineDistance(lat, lng, lat + offset, lng);
    TEST_ASSERT_DOUBLE_WITHIN(2.0, 30.0, d); // within 2m accuracy
}

void test_equator_one_degree_longitude(void) {
    // 1 degree longitude at equator ≈ 111,320 m
    double d = haversineDistance(0.0, 0.0, 0.0, 1.0);
    TEST_ASSERT_DOUBLE_WITHIN(500.0, 111320.0, d);
}

void test_symmetry(void) {
    double d1 = haversineDistance(59.0, 10.0, 60.0, 11.0);
    double d2 = haversineDistance(60.0, 11.0, 59.0, 10.0);
    TEST_ASSERT_DOUBLE_WITHIN(0.001, d1, d2);
}

void test_negative_coordinates(void) {
    // Southern hemisphere
    double d = haversineDistance(-33.8688, 151.2093, -33.8688, 151.2093);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, d);
}

// ======== AnchorState Tests ========

void test_initial_state(void) {
    AnchorState state;
    TEST_ASSERT_FALSE(state.anchorSet);
    TEST_ASSERT_FALSE(state.alarmActive);
    TEST_ASSERT_FALSE(state.alarmSilenced);
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 30.0, state.alarmRadius);
    TEST_ASSERT_FALSE(state.shouldBuzzerSound());
}

void test_set_anchor(void) {
    AnchorState state;
    state.setAnchor(59.0, 10.0);
    TEST_ASSERT_TRUE(state.anchorSet);
    TEST_ASSERT_DOUBLE_WITHIN(0.0001, 59.0, state.anchorLat);
    TEST_ASSERT_DOUBLE_WITHIN(0.0001, 10.0, state.anchorLng);
    TEST_ASSERT_FALSE(state.alarmActive);
}

void test_position_within_radius_no_alarm(void) {
    AnchorState state;
    state.setAnchor(59.0, 10.0);
    // Move ~10m north (well within 30m default)
    double offset = 10.0 / 111320.0;
    state.updatePosition(59.0 + offset, 10.0);
    TEST_ASSERT_FALSE(state.alarmActive);
    TEST_ASSERT_FALSE(state.shouldBuzzerSound());
    TEST_ASSERT_DOUBLE_WITHIN(2.0, 10.0, state.currentDistance);
}

void test_position_outside_radius_triggers_alarm(void) {
    AnchorState state;
    state.setAnchor(59.0, 10.0);
    // Move ~50m north (outside 30m default)
    double offset = 50.0 / 111320.0;
    state.updatePosition(59.0 + offset, 10.0);
    TEST_ASSERT_TRUE(state.alarmActive);
    TEST_ASSERT_TRUE(state.shouldBuzzerSound());
}

void test_alarm_returns_within_radius(void) {
    AnchorState state;
    state.setAnchor(59.0, 10.0);

    // Drift outside
    double offset = 50.0 / 111320.0;
    state.updatePosition(59.0 + offset, 10.0);
    TEST_ASSERT_TRUE(state.alarmActive);

    // Return to anchor
    state.updatePosition(59.0, 10.0);
    TEST_ASSERT_FALSE(state.alarmActive);
    TEST_ASSERT_FALSE(state.shouldBuzzerSound());
}

void test_silence_alarm(void) {
    AnchorState state;
    state.setAnchor(59.0, 10.0);
    double offset = 50.0 / 111320.0;
    state.updatePosition(59.0 + offset, 10.0);
    TEST_ASSERT_TRUE(state.shouldBuzzerSound());

    state.silenceAlarm();
    TEST_ASSERT_TRUE(state.alarmActive);       // still drifting
    TEST_ASSERT_FALSE(state.shouldBuzzerSound()); // but silenced
}

void test_silence_resets_when_returning(void) {
    AnchorState state;
    state.setAnchor(59.0, 10.0);
    double offset = 50.0 / 111320.0;

    state.updatePosition(59.0 + offset, 10.0);
    state.silenceAlarm();
    TEST_ASSERT_TRUE(state.alarmSilenced);

    // Return within radius
    state.updatePosition(59.0, 10.0);
    TEST_ASSERT_FALSE(state.alarmSilenced); // silence cleared
}

void test_update_without_anchor_set(void) {
    AnchorState state;
    bool changed = state.updatePosition(59.0, 10.0);
    TEST_ASSERT_FALSE(changed);
    TEST_ASSERT_FALSE(state.alarmActive);
}

void test_set_valid_radius(void) {
    AnchorState state;
    TEST_ASSERT_TRUE(state.setRadius(50.0));
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 50.0, state.alarmRadius);
}

void test_reject_radius_too_small(void) {
    AnchorState state;
    TEST_ASSERT_FALSE(state.setRadius(2.0));
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 30.0, state.alarmRadius); // unchanged
}

void test_reject_radius_too_large(void) {
    AnchorState state;
    TEST_ASSERT_FALSE(state.setRadius(1000.0));
    TEST_ASSERT_DOUBLE_WITHIN(0.1, 30.0, state.alarmRadius); // unchanged
}

void test_radius_boundary_min(void) {
    AnchorState state;
    TEST_ASSERT_TRUE(state.setRadius(5.0));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 5.0, state.alarmRadius);
}

void test_radius_boundary_max(void) {
    AnchorState state;
    TEST_ASSERT_TRUE(state.setRadius(500.0));
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 500.0, state.alarmRadius);
}

void test_state_change_detection(void) {
    AnchorState state;
    state.setAnchor(59.0, 10.0);

    // Same position — no change
    bool changed1 = state.updatePosition(59.0, 10.0);
    TEST_ASSERT_FALSE(changed1);

    // Drift out — alarm changes to active
    double offset = 50.0 / 111320.0;
    bool changed2 = state.updatePosition(59.0 + offset, 10.0);
    TEST_ASSERT_TRUE(changed2);

    // Still outside — no change (already active)
    bool changed3 = state.updatePosition(59.0 + offset, 10.0);
    TEST_ASSERT_FALSE(changed3);

    // Return — alarm changes to inactive
    bool changed4 = state.updatePosition(59.0, 10.0);
    TEST_ASSERT_TRUE(changed4);
}

void test_re_anchor_clears_alarm(void) {
    AnchorState state;
    state.setAnchor(59.0, 10.0);
    double offset = 50.0 / 111320.0;
    state.updatePosition(59.0 + offset, 10.0);
    TEST_ASSERT_TRUE(state.alarmActive);

    // Re-anchor at new position
    state.setAnchor(59.0 + offset, 10.0);
    TEST_ASSERT_FALSE(state.alarmActive);
    TEST_ASSERT_FALSE(state.alarmSilenced);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.0, state.currentDistance);
}

// ======== Unity Required Setup/Teardown ========
void setUp(void) {}
void tearDown(void) {}

// ======== Test Runner ========

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Haversine tests
    RUN_TEST(test_same_point_distance_is_zero);
    RUN_TEST(test_known_distance_oslo_to_copenhagen);
    RUN_TEST(test_short_distance_30m);
    RUN_TEST(test_equator_one_degree_longitude);
    RUN_TEST(test_symmetry);
    RUN_TEST(test_negative_coordinates);

    // AnchorState tests
    RUN_TEST(test_initial_state);
    RUN_TEST(test_set_anchor);
    RUN_TEST(test_position_within_radius_no_alarm);
    RUN_TEST(test_position_outside_radius_triggers_alarm);
    RUN_TEST(test_alarm_returns_within_radius);
    RUN_TEST(test_silence_alarm);
    RUN_TEST(test_silence_resets_when_returning);
    RUN_TEST(test_update_without_anchor_set);
    RUN_TEST(test_set_valid_radius);
    RUN_TEST(test_reject_radius_too_small);
    RUN_TEST(test_reject_radius_too_large);
    RUN_TEST(test_radius_boundary_min);
    RUN_TEST(test_radius_boundary_max);
    RUN_TEST(test_state_change_detection);
    RUN_TEST(test_re_anchor_clears_alarm);

    return UNITY_END();
}
