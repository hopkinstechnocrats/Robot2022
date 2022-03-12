package lib;

import org.junit.*;

import static org.junit.Assert.*;

class LinearInterpolatorTest {

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
        assertEquals(testInterpolator.get(-1), 1, 0);
        assertEquals(testInterpolator.get(21), 40, 0);
    }

    @Test
    public void testInBounds() {
        assertEquals(testInterpolator.get(0.5), (1+3)/2., 0);
        assertEquals(testInterpolator.get(7), (2/5.)*(10)+(3/5.)*(20), 0);
    }

    @Test
    public void testOnKey() {
        assertEquals(testInterpolator.get(5), 10, 0);
        assertEquals(testInterpolator.get(20), 40, 0);
        assertEquals(testInterpolator.get(0), 1, 0);
    }


    @After
    void tearDown() {
        testInterpolator = null;
    }
}