package org.firstinspires.ftc.teamcode.utilities.datastructures;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;

import java.util.LinkedList;

public class PoseQueue {

    private int elementCapacity;

    private LinkedList<PoseElement> elements;
    private RobotEx robotInstance;

    public PoseQueue(int elementCapacity) {
        this.elementCapacity = elementCapacity;

        this.elements = new LinkedList<>();

        this.robotInstance = RobotEx.getInstance();
    }


    public void addElement(Pose element) {

        PoseElement elementObject = new PoseElement(element, robotInstance.runTime);

        elements.add(elementObject);

        if (elements.size() > elementCapacity) {
            elements.removeFirst();
        }
    }

    public Pose getElementFromTimestamp(double timestamp) {

        if (elements.size() == 0) {
            System.out.println("[Pose Queue]: Tried to obtain data when list is empty.");
            return null;
        };
        PoseElement targetElement = null;
        PoseElement previousElement = elements.getFirst();

        for (PoseElement currentElement : elements) {
            if (currentElement.getTimestamp() > timestamp) {
                targetElement = previousElement;
                break;
            }
        }

        if (targetElement == null) {
            targetElement = elements.getLast();
        }

        return targetElement.getElement();
    }


}
