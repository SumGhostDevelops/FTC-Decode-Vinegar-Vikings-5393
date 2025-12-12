package org.firstinspires.ftc.teamcode.controls;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class InputHandler {
    // A simple container for a trigger condition and the resulting action
    private static class Binding {
        BooleanSupplier condition;
        Runnable action;

        Binding(BooleanSupplier condition, Runnable action) {
            this.condition = condition;
            this.action = action;
        }
    }

    private final List<Binding> bindings = new ArrayList<>();

    /**
     * Bind an action to a condition.
     * @param condition A function that returns true when the action should run.
     * @param action The code to execute.
     */
    public void bind(BooleanSupplier condition, Runnable action) {
        bindings.add(new Binding(condition, action));
    }

    /**
     * Call this once per loop inside your OpMode loop().
     */
    public void update() {
        for (Binding binding : bindings) {
            if (binding.condition.getAsBoolean()) {
                binding.action.run();
            }
        }
    }

    // Optional: Clear bindings if you change OpModes
    public void clear() {
        bindings.clear();
    }
}