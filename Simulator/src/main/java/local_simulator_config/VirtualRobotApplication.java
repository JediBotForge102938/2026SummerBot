package virtual_robot.controller;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.ComboBox;
import javafx.scene.control.TextArea;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Priority;
import javafx.scene.layout.VBox;
import javafx.stage.Stage;

/** Root-project desktop layout override; the field and simulator controller remain upstream. */
public class VirtualRobotApplication extends Application {
    private static VirtualRobotController controllerHandle;

    @Override
    public void start(Stage stage) throws Exception {
        FXMLLoader loader = new FXMLLoader(getClass().getResource("virtual_robot.fxml"));
        BorderPane root = loader.load();
        controllerHandle = loader.getController();
        configureLayout(root);
        stage.setTitle("Virtual Robot");
        stage.setScene(new Scene(root));
        stage.setResizable(true);
        stage.setOnShowing(event -> controllerHandle.setConfig(null));
        stage.show();
    }

    public static void configureLayout(BorderPane root) {
        VBox sidebar = (VBox) root.getRight();
        sidebar.setAlignment(javafx.geometry.Pos.TOP_CENTER);
        sidebar.setMinWidth(320);
        sidebar.setPrefWidth(420);
        sidebar.setMaxWidth(Double.MAX_VALUE);
        // Extra horizontal space belongs to telemetry, not to the fixed-size field.
        sidebar.prefWidthProperty().bind(javafx.beans.binding.Bindings.max(420,
                root.widthProperty().subtract(
                        root.getCenter().prefWidth(-1) + root.getLeft().prefWidth(-1))));
        for (javafx.scene.Node child : sidebar.getChildren()) {
            if (child instanceof TextArea) {
                TextArea telemetry = (TextArea) child;
                telemetry.setWrapText(true);
                telemetry.setMinHeight(200);
                telemetry.setPrefHeight(480);
                telemetry.setMaxHeight(Double.MAX_VALUE);
                telemetry.setMaxWidth(Double.MAX_VALUE);
                VBox.setVgrow(telemetry, Priority.ALWAYS);
            } else if (child instanceof ComboBox<?>) {
                ((ComboBox<?>) child).setMaxWidth(Double.MAX_VALUE);
            }
        }
    }

    @Override
    public void stop() {
        if (controllerHandle == null) return;
        if (controllerHandle.executorService != null && !controllerHandle.executorService.isShutdown()) {
            controllerHandle.executorService.shutdownNow();
        }
        if (controllerHandle.gamePadExecutorService != null
                && !controllerHandle.gamePadExecutorService.isShutdown()) {
            controllerHandle.gamePadExecutorService.shutdownNow();
        }
        controllerHandle.gamePadHelper.quit();
    }

    public static VirtualRobotController getControllerHandle() {
        return controllerHandle;
    }

    public static void main(String[] args) {
        launch(args);
    }
}
