package lib;

import org.junit.*;

import static org.junit.Assert.*;

public class LinearInterpolatorTest {

    LinearInterpolator testInterpolator;

    @Before
    public void setUp() {
        testInterpolator = new LinearInterpolator();
        testInterpolator.put(0,1);
        testInterpolator.put(1,3);
        testInterpolator.put(10, 20);
        testInterpolator.put(5, 10);
        testInterpolator.put(20, 40);
    }

    @Test
    public void testOutOfBounds() {
        assertEquals(1, testInterpolator.get(-1), 0);
        assertEquals(40, testInterpolator.get(21), 0);
    }

    @Test
    public void testInBounds() {
        assertEquals(2, testInterpolator.get(0.5), 0);
        assertEquals((2/5.)*(10)+(3/5.)*(20), testInterpolator.get(7), 0);
    }

    @Test
    public void testOnKey() {
        assertEquals(10, testInterpolator.get(5), 0);
        assertEquals(40, testInterpolator.get(20), 0);
        assertEquals(1, testInterpolator.get(0), 0);
    }


    @After
    public void tearDown() {
        testInterpolator = null;
    }
}