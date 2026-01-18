package frc.robot.util;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import javax.swing.*;

/**
 * Interactive visualizer for testing the TurretAimingCalculator algorithm.
 * Implements the same algorithm as TurretAimingCalculator for standalone testing.
 *
 * This visualizer mirrors the exact calculation logic from TurretAimingCalculator.java
 * to verify the algorithm works correctly.
 *
 * Run with: java -cp build/classes/java/main frc.robot.util.TurretVisualizerApp
 * Or just open tools/turret-visualizer.html in a browser for the same functionality.
 */
public class TurretVisualizerApp extends JFrame {
    // Field dimensions in meters (FRC 2024 field)
    private static final double FIELD_WIDTH = 16.54;
    private static final double FIELD_HEIGHT = 8.02;

    // Constants from TurretAimingConstants (mirrored here for standalone operation)
    private static final double[] BLUE_TARGET = {0.0, 5.55};
    private static final double[] RED_TARGET = {16.54, 5.55};
    private static final double TURRET_OFFSET_X = 0.0;
    private static final double TURRET_OFFSET_Y = 0.0;
    private static final double TURRET_ZERO_ANGLE = 0.0;
    private static final double MIN_SHOOTING_DISTANCE = 1.0;
    private static final double MAX_SHOOTING_DISTANCE = 10.0;

    // Display settings
    private static final int CANVAS_WIDTH = 900;
    private static final int CANVAS_HEIGHT = (int) (CANVAS_WIDTH * FIELD_HEIGHT / FIELD_WIDTH);
    private static final double SCALE = CANVAS_WIDTH / FIELD_WIDTH;

    // Robot state
    private double robotX = 8.0;
    private double robotY = 4.0;
    private double robotRotationDeg = 0.0;

    // Target settings
    private double[] customTarget = null;
    private boolean useRedAlliance = false;
    private double turretOffsetX = TURRET_OFFSET_X;
    private double turretOffsetY = TURRET_OFFSET_Y;
    private double turretZeroAngle = TURRET_ZERO_ANGLE;

    // Calculation result
    private AimingResult lastResult;

    // UI components
    private FieldPanel fieldPanel;
    private JLabel angleLabel, distanceLabel, reachableLabel;
    private TurretDial turretDial;

    /**
     * Result record matching TurretAimingCalculator.AimingResult
     */
    record AimingResult(
            double turretAngleDegrees,
            double distanceMeters,
            boolean isReachable,
            double[] turretPosition,
            double[] targetPosition) {
    }

    public TurretVisualizerApp() {
        super("Turret Aiming Calculator Visualizer");
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        createUI();
        pack();
        setMinimumSize(new Dimension(1250, 750));
        setLocationRelativeTo(null);
        updateCalculation();
    }

    /**
     * Calculates turret aiming - mirrors TurretAimingCalculator.calculate() exactly
     */
    private AimingResult calculateAiming() {
        // Get target position based on alliance
        double[] target = customTarget != null ? customTarget :
            (useRedAlliance ? RED_TARGET : BLUE_TARGET);

        // Robot pose
        double robotRotationRad = Math.toRadians(robotRotationDeg);

        // Account for turret offset from robot center
        // This matches: turretPosition = robotPose.getTranslation()
        //     .plus(new Translation2d(TURRET_OFFSET_X, TURRET_OFFSET_Y)
        //     .rotateBy(robotPose.getRotation()));
        double offsetRotatedX = turretOffsetX * Math.cos(robotRotationRad) - turretOffsetY * Math.sin(robotRotationRad);
        double offsetRotatedY = turretOffsetX * Math.sin(robotRotationRad) + turretOffsetY * Math.cos(robotRotationRad);
        double turretX = robotX + offsetRotatedX;
        double turretY = robotY + offsetRotatedY;

        // Calculate distance to target
        double dx = target[0] - turretX;
        double dy = target[1] - turretY;
        double distance = Math.sqrt(dx * dx + dy * dy);

        // Calculate field-relative angle to target
        // This matches: Math.atan2(targetPosition.getY() - turretPosition.getY(),
        //                          targetPosition.getX() - turretPosition.getX())
        double fieldAngleRadians = Math.atan2(dy, dx);

        // Convert to robot-relative angle (subtract robot heading)
        // This matches: robotRelativeRadians = fieldAngleRadians - robotPose.getRotation().getRadians()
        double robotRelativeRadians = fieldAngleRadians - robotRotationRad;

        // Account for turret zero angle offset
        // This matches: robotRelativeRadians -= Math.toRadians(TURRET_ZERO_ANGLE_DEGREES)
        robotRelativeRadians -= Math.toRadians(turretZeroAngle);

        // Normalize to [0, 360) degrees
        // This matches the normalization in TurretAimingCalculator
        double turretAngleDegrees = Math.toDegrees(robotRelativeRadians) % 360.0;
        if (turretAngleDegrees < 0) {
            turretAngleDegrees += 360.0;
        }

        // Check if target is within valid shooting range
        boolean isReachable = distance >= MIN_SHOOTING_DISTANCE && distance <= MAX_SHOOTING_DISTANCE;

        return new AimingResult(turretAngleDegrees, distance, isReachable,
                new double[]{turretX, turretY}, target);
    }

    private void createUI() {
        setLayout(new BorderLayout(10, 10));
        ((JPanel) getContentPane()).setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));
        getContentPane().setBackground(new Color(26, 26, 46));

        // Field panel (center) - wrapped in container to keep fixed size and centered
        fieldPanel = new FieldPanel();
        JPanel fieldContainer = new JPanel(new GridBagLayout());
        fieldContainer.setBackground(new Color(26, 26, 46));
        fieldContainer.add(fieldPanel);
        add(fieldContainer, BorderLayout.CENTER);

        // Control panel (right side)
        JPanel controlPanel = createControlPanel();
        add(controlPanel, BorderLayout.EAST);
    }

    private JPanel createControlPanel() {
        JPanel panel = new JPanel();
        panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
        panel.setBackground(new Color(22, 33, 62));
        panel.setBorder(BorderFactory.createEmptyBorder(10, 15, 10, 15));
        panel.setPreferredSize(new Dimension(320, 0));

        // Robot Position section
        panel.add(createSectionLabel("Robot Position"));

        // X control
        panel.add(createSliderRow("X (m):", 0, FIELD_WIDTH, robotX, v -> {
            robotX = v;
            updateCalculation();
        }));

        // Y control
        panel.add(createSliderRow("Y (m):", 0, FIELD_HEIGHT, robotY, v -> {
            robotY = v;
            updateCalculation();
        }));

        // Rotation control
        panel.add(createSliderRow("Rotation (°):", -180, 180, robotRotationDeg, v -> {
            robotRotationDeg = v;
            updateCalculation();
        }));

        panel.add(Box.createVerticalStrut(15));

        // Alliance selector
        panel.add(createSectionLabel("Alliance"));
        JComboBox<String> allianceCombo = new JComboBox<>(new String[]{"Blue Alliance", "Red Alliance"});
        allianceCombo.setMaximumSize(new Dimension(Integer.MAX_VALUE, 30));
        allianceCombo.addActionListener(e -> {
            useRedAlliance = allianceCombo.getSelectedIndex() == 1;
            updateCalculation();
        });
        styleComponent(allianceCombo);
        panel.add(allianceCombo);

        panel.add(Box.createVerticalStrut(15));

        // Custom target section
        panel.add(createSectionLabel("Custom Target (optional)"));
        JPanel targetPanel = new JPanel(new GridLayout(2, 2, 5, 5));
        targetPanel.setBackground(new Color(22, 33, 62));
        targetPanel.setMaximumSize(new Dimension(Integer.MAX_VALUE, 60));

        JTextField targetXField = new JTextField();
        JTextField targetYField = new JTextField();
        styleTextField(targetXField);
        styleTextField(targetYField);

        targetPanel.add(createSmallLabel("X:"));
        targetPanel.add(targetXField);
        targetPanel.add(createSmallLabel("Y:"));
        targetPanel.add(targetYField);
        panel.add(targetPanel);

        JButton setTargetBtn = new JButton("Set Custom Target");
        styleButton(setTargetBtn);
        setTargetBtn.addActionListener(e -> {
            try {
                double tx = Double.parseDouble(targetXField.getText());
                double ty = Double.parseDouble(targetYField.getText());
                customTarget = new double[]{tx, ty};
                updateCalculation();
            } catch (NumberFormatException ex) {
                customTarget = null;
                updateCalculation();
            }
        });
        panel.add(setTargetBtn);

        JButton clearTargetBtn = new JButton("Clear Custom Target");
        styleButton(clearTargetBtn);
        clearTargetBtn.addActionListener(e -> {
            customTarget = null;
            targetXField.setText("");
            targetYField.setText("");
            updateCalculation();
        });
        panel.add(clearTargetBtn);

        panel.add(Box.createVerticalStrut(20));

        // Results section
        panel.add(createSectionLabel("Calculation Results"));

        JPanel resultsPanel = new JPanel(new GridLayout(3, 2, 5, 8));
        resultsPanel.setBackground(new Color(15, 52, 96));
        resultsPanel.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10));

        angleLabel = createResultLabel("0.00°");
        distanceLabel = createResultLabel("0.00 m");
        reachableLabel = createResultLabel("-");

        resultsPanel.add(createSmallLabel("Turret Angle:"));
        resultsPanel.add(angleLabel);
        resultsPanel.add(createSmallLabel("Distance:"));
        resultsPanel.add(distanceLabel);
        resultsPanel.add(createSmallLabel("Reachable:"));
        resultsPanel.add(reachableLabel);

        panel.add(resultsPanel);

        panel.add(Box.createVerticalStrut(15));

        // Turret dial visualization
        turretDial = new TurretDial();
        turretDial.setPreferredSize(new Dimension(150, 150));
        turretDial.setMaximumSize(new Dimension(150, 150));
        turretDial.setAlignmentX(Component.CENTER_ALIGNMENT);
        panel.add(turretDial);

        panel.add(Box.createVerticalStrut(15));

        // Buttons
        JButton resetBtn = new JButton("Reset Position");
        styleButton(resetBtn);
        resetBtn.addActionListener(e -> {
            robotX = 8.0;
            robotY = 4.0;
            robotRotationDeg = 0;
            updateCalculation();
        });
        panel.add(resetBtn);

        JButton testBtn = new JButton("Run Test Cases");
        styleButton(testBtn);
        testBtn.addActionListener(e -> runTestCases());
        panel.add(testBtn);

        panel.add(Box.createVerticalGlue());

        // Instructions
        panel.add(createSectionLabel("Instructions"));
        JTextArea instructions = new JTextArea(
            "• Drag robot on field to move\n" +
            "• Right-click + drag to rotate\n" +
            "• Use sliders for precise control\n" +
            "• Green line = turret aim direction\n" +
            "• Mirrors TurretAimingCalculator algorithm"
        );
        instructions.setEditable(false);
        instructions.setBackground(new Color(15, 52, 96));
        instructions.setForeground(new Color(170, 170, 170));
        instructions.setFont(new Font("SansSerif", Font.PLAIN, 11));
        instructions.setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));
        panel.add(instructions);

        return panel;
    }

    private JPanel createSliderRow(String label, double min, double max, double initial,
                                    java.util.function.DoubleConsumer onChange) {
        JPanel row = new JPanel(new BorderLayout(5, 0));
        row.setBackground(new Color(22, 33, 62));
        row.setMaximumSize(new Dimension(Integer.MAX_VALUE, 35));

        JLabel lbl = new JLabel(label);
        lbl.setForeground(new Color(170, 170, 170));
        lbl.setPreferredSize(new Dimension(90, 20));
        row.add(lbl, BorderLayout.WEST);

        JSlider slider = new JSlider((int)(min * 100), (int)(max * 100), (int)(initial * 100));
        slider.setBackground(new Color(22, 33, 62));
        slider.setForeground(Color.WHITE);

        JSpinner spinner = new JSpinner(new SpinnerNumberModel(initial, min, max, 0.1));
        spinner.setPreferredSize(new Dimension(70, 25));
        styleComponent(spinner);

        slider.addChangeListener(e -> {
            double val = slider.getValue() / 100.0;
            spinner.setValue(val);
            onChange.accept(val);
        });

        spinner.addChangeListener(e -> {
            double val = (Double) spinner.getValue();
            slider.setValue((int)(val * 100));
            onChange.accept(val);
        });

        row.add(slider, BorderLayout.CENTER);
        row.add(spinner, BorderLayout.EAST);

        return row;
    }

    private JLabel createSectionLabel(String text) {
        JLabel label = new JLabel(text);
        label.setForeground(new Color(0, 212, 255));
        label.setFont(new Font("SansSerif", Font.BOLD, 14));
        label.setBorder(BorderFactory.createMatteBorder(0, 0, 1, 0, new Color(15, 52, 96)));
        label.setAlignmentX(Component.LEFT_ALIGNMENT);
        return label;
    }

    private JLabel createSmallLabel(String text) {
        JLabel label = new JLabel(text);
        label.setForeground(new Color(136, 136, 136));
        return label;
    }

    private JLabel createResultLabel(String text) {
        JLabel label = new JLabel(text);
        label.setForeground(Color.WHITE);
        label.setFont(new Font("Monospaced", Font.BOLD, 14));
        return label;
    }

    private void styleButton(JButton btn) {
        btn.setBackground(new Color(0, 212, 255));
        btn.setForeground(new Color(26, 26, 46));
        btn.setFocusPainted(false);
        btn.setBorderPainted(false);
        btn.setFont(new Font("SansSerif", Font.BOLD, 12));
        btn.setMaximumSize(new Dimension(Integer.MAX_VALUE, 35));
        btn.setAlignmentX(Component.CENTER_ALIGNMENT);
        btn.setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
    }

    private void styleComponent(JComponent comp) {
        comp.setBackground(new Color(26, 26, 46));
        comp.setForeground(Color.WHITE);
    }

    private void styleTextField(JTextField field) {
        field.setBackground(new Color(26, 26, 46));
        field.setForeground(Color.WHITE);
        field.setCaretColor(Color.WHITE);
        field.setBorder(BorderFactory.createLineBorder(new Color(15, 52, 96)));
    }

    private void updateCalculation() {
        // Calculate using our local implementation (mirrors TurretAimingCalculator)
        lastResult = calculateAiming();

        // Update UI
        angleLabel.setText(String.format("%.2f°", lastResult.turretAngleDegrees()));
        distanceLabel.setText(String.format("%.2f m", lastResult.distanceMeters()));

        if (lastResult.isReachable()) {
            reachableLabel.setText("YES");
            reachableLabel.setForeground(new Color(0, 255, 136));
        } else {
            reachableLabel.setText("NO");
            reachableLabel.setForeground(new Color(255, 68, 68));
        }

        turretDial.setAngle(lastResult.turretAngleDegrees());
        fieldPanel.repaint();
    }

    /**
     * Test case record for visualization
     */
    private record TestCase(double x, double y, double rotation, boolean redAlliance, String description) {}

    private void runTestCases() {
        // Test cases extracted from TurretAimingCalculatorTest.java
        TestCase[] testCases = {
            // Angle calculation tests
            new TestCase(5.0, 5.55, 180, false, "Robot facing target (turret = 0°)"),
            new TestCase(5.0, 5.55, 0, false, "Target behind robot (turret = 180°)"),
            new TestCase(0.0, 0.0, 90, false, "Target directly ahead (turret = 0°)"),
            new TestCase(5.0, 3.0, 0, false, "Angle normalization test"),

            // Robot rotation tests
            new TestCase(8.0, 4.0, 0, false, "Robot rotation: 0°"),
            new TestCase(8.0, 4.0, 45, false, "Robot rotation: 45°"),
            new TestCase(8.0, 4.0, 90, false, "Robot rotation: 90°"),
            new TestCase(8.0, 4.0, 135, false, "Robot rotation: 135°"),
            new TestCase(8.0, 4.0, 180, false, "Robot rotation: 180°"),

            // Distance tests
            new TestCase(3.0, 5.55, 0, false, "Distance test: 3.0m from target"),
            new TestCase(3.0, 1.55, 0, false, "Diagonal distance: 5.0m (3-4-5 triangle)"),

            // Reachability tests
            new TestCase(5.0, 5.55, 0, false, "Reachable: 5.0m (in range)"),
            new TestCase(1.0, 5.55, 0, false, "Too close: 1.0m (unreachable)"),
            new TestCase(12.0, 5.55, 0, false, "Too far: 12.0m (unreachable)"),

            // Red alliance test
            new TestCase(11.54, 5.55, 0, true, "Red alliance: aiming at red target"),

            // Integration tests
            new TestCase(5.0, 4.0, 45, false, "Full workflow: realistic position"),
            new TestCase(6.0, 3.0, 30, false, "Consistency test position"),
        };

        new Thread(() -> {
            for (int i = 0; i < testCases.length; i++) {
                TestCase test = testCases[i];
                final int testNum = i + 1;
                final int total = testCases.length;

                robotX = test.x();
                robotY = test.y();
                robotRotationDeg = test.rotation();
                useRedAlliance = test.redAlliance();

                SwingUtilities.invokeLater(() -> {
                    updateCalculation();
                    setTitle(String.format("Test %d/%d: %s", testNum, total, test.description()));
                });

                System.out.printf("Test %d/%d: %s%n", testNum, total, test.description());
                System.out.printf("  Position: (%.2f, %.2f) @ %.0f° | Alliance: %s%n",
                    test.x(), test.y(), test.rotation(), test.redAlliance() ? "Red" : "Blue");
                System.out.printf("  Result: Turret=%.2f°, Distance=%.2fm, Reachable=%s%n%n",
                    lastResult.turretAngleDegrees(), lastResult.distanceMeters(),
                    lastResult.isReachable() ? "YES" : "NO");

                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    break;
                }
            }

            // Reset to default
            robotX = 8.0;
            robotY = 4.0;
            robotRotationDeg = 0;
            useRedAlliance = false;
            SwingUtilities.invokeLater(() -> {
                updateCalculation();
                setTitle("Turret Aiming Calculator Visualizer");
            });
        }).start();
    }

    /**
     * Custom panel for drawing the field
     */
    private class FieldPanel extends JPanel {
        private boolean isDragging = false;
        private boolean isRotating = false;
        private Point dragStart;
        private double rotationStart;

        public FieldPanel() {
            Dimension fixedSize = new Dimension(CANVAS_WIDTH, CANVAS_HEIGHT);
            setPreferredSize(fixedSize);
            setMinimumSize(fixedSize);
            setMaximumSize(fixedSize);
            setBackground(new Color(45, 90, 39));

            addMouseListener(new MouseAdapter() {
                @Override
                public void mousePressed(MouseEvent e) {
                    if (SwingUtilities.isRightMouseButton(e)) {
                        isRotating = true;
                        dragStart = e.getPoint();
                        rotationStart = robotRotationDeg;
                    } else {
                        isDragging = true;
                        Point2D.Double meters = canvasToMeters(e.getPoint());
                        robotX = clamp(meters.x, 0, FIELD_WIDTH);
                        robotY = clamp(meters.y, 0, FIELD_HEIGHT);
                        updateCalculation();
                    }
                }

                @Override
                public void mouseReleased(MouseEvent e) {
                    isDragging = false;
                    isRotating = false;
                }
            });

            addMouseMotionListener(new MouseMotionAdapter() {
                @Override
                public void mouseDragged(MouseEvent e) {
                    if (isDragging) {
                        Point2D.Double meters = canvasToMeters(e.getPoint());
                        robotX = clamp(meters.x, 0, FIELD_WIDTH);
                        robotY = clamp(meters.y, 0, FIELD_HEIGHT);
                        updateCalculation();
                    } else if (isRotating) {
                        double dx = e.getX() - dragStart.x;
                        robotRotationDeg = rotationStart + dx * 0.5;
                        // Normalize to [-180, 180]
                        while (robotRotationDeg > 180) robotRotationDeg -= 360;
                        while (robotRotationDeg < -180) robotRotationDeg += 360;
                        updateCalculation();
                    }
                }
            });
        }

        private Point2D.Double canvasToMeters(Point p) {
            return new Point2D.Double(
                p.x / SCALE,
                (CANVAS_HEIGHT - p.y) / SCALE
            );
        }

        private Point metersToCanvas(double x, double y) {
            return new Point(
                (int)(x * SCALE),
                (int)(CANVAS_HEIGHT - y * SCALE)
            );
        }

        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2d = (Graphics2D) g;
            g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            drawField(g2d);
            drawTargets(g2d);
            drawRobot(g2d);
            drawInfo(g2d);
        }

        private void drawField(Graphics2D g) {
            // Field background
            g.setColor(new Color(45, 90, 39));
            g.fillRect(0, 0, getWidth(), getHeight());

            // Border
            g.setColor(Color.WHITE);
            g.setStroke(new BasicStroke(2));
            g.drawRect(1, 1, getWidth() - 3, getHeight() - 3);

            // Grid
            g.setColor(new Color(255, 255, 255, 25));
            g.setStroke(new BasicStroke(1));
            for (int x = 0; x <= FIELD_WIDTH; x++) {
                Point p = metersToCanvas(x, 0);
                g.drawLine(p.x, 0, p.x, getHeight());
            }
            for (int y = 0; y <= FIELD_HEIGHT; y++) {
                Point p = metersToCanvas(0, y);
                g.drawLine(0, p.y, getWidth(), p.y);
            }

            // Alliance zones
            g.setColor(new Color(0, 100, 255, 50));
            g.fillRect(0, 0, getWidth() / 6, getHeight());
            g.setColor(new Color(255, 0, 0, 50));
            g.fillRect(getWidth() * 5 / 6, 0, getWidth() / 6, getHeight());
        }

        private void drawTargets(Graphics2D g) {
            // Blue target
            Point bluePos = metersToCanvas(BLUE_TARGET[0], BLUE_TARGET[1]);
            g.setColor(new Color(77, 171, 247));
            g.fillOval(bluePos.x - 12, bluePos.y - 12, 24, 24);
            g.setColor(Color.WHITE);
            g.setStroke(new BasicStroke(2));
            g.drawOval(bluePos.x - 12, bluePos.y - 12, 24, 24);

            // Red target
            Point redPos = metersToCanvas(RED_TARGET[0], RED_TARGET[1]);
            g.setColor(new Color(255, 107, 107));
            g.fillOval(redPos.x - 12, redPos.y - 12, 24, 24);
            g.setColor(Color.WHITE);
            g.drawOval(redPos.x - 12, redPos.y - 12, 24, 24);

            // Custom target
            if (customTarget != null) {
                Point customPos = metersToCanvas(customTarget[0], customTarget[1]);
                g.setColor(new Color(255, 212, 59));
                g.fillOval(customPos.x - 10, customPos.y - 10, 20, 20);
                g.setColor(Color.WHITE);
                g.drawOval(customPos.x - 10, customPos.y - 10, 20, 20);
            }

            // Highlight active target
            double[] activeTarget = customTarget != null ? customTarget :
                (useRedAlliance ? RED_TARGET : BLUE_TARGET);
            Point activePos = metersToCanvas(activeTarget[0], activeTarget[1]);
            g.setColor(Color.WHITE);
            g.setStroke(new BasicStroke(3, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10,
                new float[]{5, 5}, 0));
            g.drawOval(activePos.x - 18, activePos.y - 18, 36, 36);
        }

        private void drawRobot(Graphics2D g) {
            Point pos = metersToCanvas(robotX, robotY);
            int robotSize = (int)(0.8 * SCALE);
            double rotRad = -Math.toRadians(robotRotationDeg);

            AffineTransform old = g2d(g).getTransform();
            g2d(g).translate(pos.x, pos.y);
            g2d(g).rotate(rotRad);

            // Robot body
            g.setColor(new Color(52, 152, 219));
            g.fillRect(-robotSize/2, -robotSize/2, robotSize, robotSize);
            g.setColor(new Color(41, 128, 185));
            g.setStroke(new BasicStroke(3));
            g.drawRect(-robotSize/2, -robotSize/2, robotSize, robotSize);

            // Forward indicator (yellow arrow)
            g.setColor(new Color(255, 212, 59));
            int[] xPoints = {robotSize/2 + 10, robotSize/2 - 5, robotSize/2 - 5};
            int[] yPoints = {0, -10, 10};
            g.fillPolygon(xPoints, yPoints, 3);

            // Turret center
            g.setColor(new Color(231, 76, 60));
            g.fillOval(-8, -8, 16, 16);

            g2d(g).setTransform(old);

            // Draw turret aim line
            if (lastResult != null) {
                double turretAngleRad = Math.toRadians(lastResult.turretAngleDegrees());
                double worldAngle = Math.toRadians(robotRotationDeg) + turretAngleRad;
                double lineLength = lastResult.distanceMeters() * SCALE;

                int endX = pos.x + (int)(Math.cos(worldAngle) * lineLength);
                int endY = pos.y - (int)(Math.sin(worldAngle) * lineLength);

                g.setColor(lastResult.isReachable() ? new Color(0, 255, 136) : new Color(255, 68, 68));
                g.setStroke(new BasicStroke(3, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10,
                    new float[]{10, 5}, 0));
                g.drawLine(pos.x, pos.y, endX, endY);

                // Distance arc
                g.setColor(new Color(255, 255, 255, 75));
                g.setStroke(new BasicStroke(1));
                int radius = (int)(lastResult.distanceMeters() * SCALE);
                g.drawOval(pos.x - radius, pos.y - radius, radius * 2, radius * 2);
            }
        }

        private void drawInfo(Graphics2D g) {
            g.setColor(new Color(0, 0, 0, 180));
            g.fillRect(10, 10, 200, 80);

            g.setColor(Color.WHITE);
            g.setFont(new Font("Monospaced", Font.PLAIN, 14));
            g.drawString(String.format("Robot: (%.2f, %.2f)", robotX, robotY), 20, 30);
            g.drawString(String.format("Rotation: %.1f°", robotRotationDeg), 20, 50);
            if (lastResult != null) {
                g.drawString(String.format("Turret: %.2f°", lastResult.turretAngleDegrees()), 20, 70);
            }
        }

        private Graphics2D g2d(Graphics g) {
            return (Graphics2D) g;
        }
    }

    /**
     * Turret dial visualization component
     */
    private class TurretDial extends JPanel {
        private double angle = 0;

        public void setAngle(double angle) {
            this.angle = angle;
            repaint();
        }

        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2d = (Graphics2D) g;
            g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            int cx = getWidth() / 2;
            int cy = getHeight() / 2;
            int radius = Math.min(cx, cy) - 10;

            // Background circle
            g2d.setColor(new Color(26, 26, 46));
            g2d.fillOval(cx - radius, cy - radius, radius * 2, radius * 2);

            // Border
            g2d.setColor(new Color(15, 52, 96));
            g2d.setStroke(new BasicStroke(3));
            g2d.drawOval(cx - radius, cy - radius, radius * 2, radius * 2);

            // Tick marks
            g2d.setColor(new Color(100, 100, 100));
            g2d.setStroke(new BasicStroke(1));
            for (int i = 0; i < 360; i += 30) {
                double rad = Math.toRadians(i - 90);
                int x1 = cx + (int)((radius - 8) * Math.cos(rad));
                int y1 = cy + (int)((radius - 8) * Math.sin(rad));
                int x2 = cx + (int)((radius - 2) * Math.cos(rad));
                int y2 = cy + (int)((radius - 2) * Math.sin(rad));
                g2d.drawLine(x1, y1, x2, y2);
            }

            // Pointer
            double pointerRad = Math.toRadians(angle - 90);
            int pointerLength = radius - 15;
            int px = cx + (int)(pointerLength * Math.cos(pointerRad));
            int py = cy + (int)(pointerLength * Math.sin(pointerRad));

            g2d.setColor(new Color(0, 212, 255));
            g2d.setStroke(new BasicStroke(4, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            g2d.drawLine(cx, cy, px, py);

            // Center dot
            g2d.setColor(new Color(0, 212, 255));
            g2d.fillOval(cx - 8, cy - 8, 16, 16);

            // Angle label
            g2d.setColor(Color.WHITE);
            g2d.setFont(new Font("Monospaced", Font.BOLD, 11));
            String angleStr = String.format("%.1f°", angle);
            FontMetrics fm = g2d.getFontMetrics();
            g2d.drawString(angleStr, cx - fm.stringWidth(angleStr) / 2, cy + radius + 15);
        }
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            try {
                UIManager.setLookAndFeel(UIManager.getCrossPlatformLookAndFeelClassName());
            } catch (Exception e) {
                // Ignore
            }
            new TurretVisualizerApp().setVisible(true);
        });
    }
}
