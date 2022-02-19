package frc.robot;

import static org.junit.Assert.assertEquals;

public class testLibrary {
    public static final double DELTA = 1e-2;

    @org.junit.Test // marks this method as a test
    public void test_copySign() {
      assertEquals(0, Library.copySign(1, 0), DELTA);
      assertEquals(0, Library.copySign(-1, 0), DELTA);
      assertEquals(5.4, Library.copySign(1.0, 5.4), DELTA);
      assertEquals(6.2, Library.copySign(1.2, -6.2), DELTA);
      assertEquals(-.2, Library.copySign(-1.3, .2), DELTA);
      assertEquals(-33.2, Library.copySign(-2.2, -33.2), DELTA);
      assertEquals(4, Library.copySign(0, -4), DELTA);
    }
    
}
