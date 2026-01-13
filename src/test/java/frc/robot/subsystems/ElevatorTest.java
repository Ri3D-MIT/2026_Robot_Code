package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static frc.lib.UnitTestingUtil.reset;
import static frc.lib.UnitTestingUtil.setupTests;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

public class ElevatorTest {
  private Elevator elevator;

  @BeforeEach
  void setup() {
    elevator = new Elevator();
    setupTests();
  }

  @AfterEach
  void destroy() throws Exception {
    reset(elevator);
  }

  @Test
  void init() {}

  @ParameterizedTest
  @ValueSource(doubles = {0, 5, 10, 15, 20, 25})
  void goToTest(double heightInches) {
    frc.lib.Test.runUnitTest(elevator.goToTest(Inches.of(heightInches)));
  }
}
