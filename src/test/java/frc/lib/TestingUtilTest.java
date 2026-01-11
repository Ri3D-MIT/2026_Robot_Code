package frc.lib;

import static frc.lib.UnitTestingUtil.reset;
import static frc.lib.UnitTestingUtil.runToCompletion;
import static frc.lib.UnitTestingUtil.setupTests;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

public class TestingUtilTest {

  @BeforeEach
  public void setup() {
    setupTests();
  }

  @AfterEach
  public void clear() throws Exception {
    reset();
  }

  @Test
  public void enabled() {
    assertTrue(DriverStation.isEnabled());
  }

  @ParameterizedTest
  @ValueSource(doubles = {0.4, 2, 3.2, 4.03})
  public void runToCompletionTest(double timeout) {
    Command c = Commands.run(() -> {}).withTimeout(timeout);
    double startTime = Timer.getFPGATimestamp();
    runToCompletion(c);
    assertEquals(timeout, Timer.getFPGATimestamp() - startTime, 0.3);
  }
}
