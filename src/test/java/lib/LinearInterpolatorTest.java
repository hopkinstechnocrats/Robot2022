package lib;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class LinearInterpolatorTest {

    LinearInterpolator testInterpolator;

    @BeforeEach
    void setUp() {
        testInterpolator = new LinearInterpolator();
        testInterpolator.put(0,1);
        testInterpolator.put(1,3);
        testInterpolator.put(10, 20);
        testInterpolator.put(5, 10);
        testInterpolator.put(20, 40);
    }

    @Test
    void testOutOfBounds() {
        assertEquals(testInterpolator.get(-1), 1);
        assertEquals(testInterpolator.get(21), 40);
    }

    @Test
    void testInBounds() {
        assertEquals(testInterpolator.get(0.5), (1+3)/2);
        assertEquals(testInterpolator.get(7), (2/5)*(10)+(3/5)*(20));
    }

    @Test
    void testOnKey() {
        assertEquals(testInterpolator.get(5), 10);
        assertEquals(testInterpolator.get(20), 40);
        assertEquals(testInterpolator.get(0), 1);
    }


    @AfterEach
    void tearDown() {
        testInterpolator = null;
    }
}