package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.jumpypants.murphy.states.StateMachine;
import com.jumpypants.murphy.tasks.ParallelTask;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.jumpypants.murphy.tasks.WaitTask;
import com.jumpypants.murphy.util.RobotContext;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robotStates.IntakingState;
import org.firstinspires.ftc.teamcode.subSystems.Intake;
import org.firstinspires.ftc.teamcode.subSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.Transfer;
import org.firstinspires.ftc.teamcode.subSystems.Turret;


@Configurable
@Autonomous(name="(AUTO) Auton", group="Main")
public class MainAutonClose extends LinearOpMode {
    public enum Alliance {
        RED,
        BLUE
    }

    /*public static double RED_TARGET_X = 135;
    public static double RED_TARGET_Y = 135;
    public static double BLUE_STARTING_X= 27.3162853;
    public static double BLUE_TARGET_Y = ;

    public static double BLUE_STARTING_Y = 72;
    public static double BLUE_STARTING_HEADING = 0.5 * Math.PI;

    public static double BLUE_PRELOAD_X = 61;
    ;
    public static double BLUE_PRELOAD_Y = 22;
    public static double BLUE_PRELOAD_HEADING = 0.5 * Math.PI;



    public static double BLUE_FIRST_ENDPOINT_HEADING = Math.PI;

    public static Alliance alliance = Alliance.BLUE;*/
    public static double BLUE_FIRST_ENDPOINT_X = 41;
    public static double BLUE_FIRST_ENDPOINT_Y = 36;

    Alliance alliance = Alliance.BLUE; //change this sad thing

    private Path goToPreload;
    private Path goToFirstEndpoint;
    int pathState = 0;
    private MyRobot robotContext;
    Follower follower;


    @Override
    public void runOpMode() {
        MyRobot.follower = Constants.createFollower(hardwareMap);
        follower = MyRobot.follower;

        robotContext = new MyRobot(
                hardwareMap,
                telemetry,
                gamepad1,
                gamepad2,
                new Intake(hardwareMap),
                new Shooter(hardwareMap),
                new Transfer(hardwareMap),
                new Turret(hardwareMap)
        );

        MyRobot.follower = Constants.createFollower(hardwareMap);

        Follower follower = MyRobot.follower;


        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        while (opModeInInit()){
            if (gamepad2.left_bumper){
                alliance = MainAutonClose.Alliance.BLUE;
            } else if (gamepad2.right_bumper) {
                alliance = MainAutonClose.Alliance.RED;
            }

            telemetry.addLine("Alliance: " + alliance.name());

            telemetry.update();
        }
        Pose startingPose;
        robotContext.TURRET.resetEncoder();

        SequentialTask sequentialTask = autonomousPathUpdate();

        while (opModeIsActive()) {
            sequentialTask.step();
            follower.update();
        }
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(27.316, 131.650),

                                    new Pose(47.666, 95.693)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(55), Math.toRadians(180))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(47.666, 95.693),
                                    new Pose(35.596, 81.593),
                                    new Pose(16.937, 83.896)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.937, 83.896),
                                    new Pose(35.676, 81.575),
                                    new Pose(47.826, 95.673)
                            )
                    ).setTangentHeadingInterpolation()
//                    .setReversed(true)
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(47.826, 95.673),
                                    new Pose(45.561, 55.680),
                                    new Pose(17.526, 59.349)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(17.526, 59.349),
                                    new Pose(45.528, 55.826),
                                    new Pose(47.824, 95.716)
                            )
                    ).setTangentHeadingInterpolation()
//                    .setReversed(true)
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(47.824, 95.716),
                                    new Pose(51.950, 29.084),
                                    new Pose(17.904, 35.588)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.904, 35.588),

                                    new Pose(71.941, 23.163)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(71.941, 23.163),
                                    new Pose(86.470, 33.371),
                                    new Pose(105.388, 33.510)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }
    
    private Task goToPath(PathChain path) {
        follower.followPath(path);
        follower.update();
        return new WaitTask(robotContext, 0);
    }


    private SequentialTask autonomousPathUpdate() {
        Paths newpaths = new Paths(follower);
        SequentialTask sequentialTask = new SequentialTask(
                robotContext,
                goToPath(newpaths.Path1),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, 1),
                robotContext.TRANSFER.new SendThreeTask(robotContext),
                new WaitTask(robotContext, 1),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, Shooter.IDLE_VEL),


                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                goToPath(newpaths.Path2),
                robotContext.INTAKE.new SetIntakePower(robotContext, 0),
                
                goToPath(newpaths.Path3),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, 1),
                robotContext.TRANSFER.new SendThreeTask(robotContext),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, Shooter.IDLE_VEL),
                
                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                goToPath(newpaths.Path4),
                robotContext.INTAKE.new SetIntakePower(robotContext, 0),
                
                goToPath(newpaths.Path5),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, 1),
                robotContext.TRANSFER.new SendThreeTask(robotContext),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, Shooter.IDLE_VEL),
                //Path 6 intakes, Path 7 outakes from far outake
                robotContext.INTAKE.new SetIntakePower(robotContext, 1),
                goToPath(newpaths.Path6),
                robotContext.INTAKE.new SetIntakePower(robotContext, 0),
                
                goToPath(newpaths.Path7),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, 1),
                robotContext.TRANSFER.new SendThreeTask(robotContext),
                robotContext.SHOOTER.new RunOuttakeTask(robotContext, 0)//,
                
//                goToPath(newpaths.Path8)
        );
        return sequentialTask;
    }
}
