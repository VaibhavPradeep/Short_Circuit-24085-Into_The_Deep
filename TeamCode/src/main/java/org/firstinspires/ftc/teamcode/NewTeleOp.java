package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;
public class NewTeleOp extends OpMode{
    // TODO: find transfer elapse time
    double transferElapseTime = 0;
    ElapsedTime timer = new ElapsedTime();
    public enum MotifCode{
        NONE,
        TAG_ID_1_GPP,
        TAG_ID_2_PGP,
        TAG_ID_3_PPG

    };

    enum Motif1GPP{
        G,
        P1,
        P2
    }

    enum Motif2PGP{
        P1,
        G,
        P2
    }

    enum Motif3PPG{
        P1,
        P2,
        G
    }

    MotifCode motifCode = MotifCode.NONE;
    Motif1GPP motif1 = Motif1GPP.G;
    Motif2PGP motif2 = Motif2PGP.P1;
    Motif3PPG motif3 = Motif3PPG.P1;

    MecanumDriveCirc drivetrain = new MecanumDriveCirc();
    NewILC ILC = new NewILC();

    private final int READ_PERIOD = 1;



    /*
    FINISHED: DT: 4 motors
    TODO: 1 motor for itnake
    TODO: 1 CR servo for sorter
    TODO: 1 servo for transfer wheels

    TODO: Turret: 3 Motors
    TODO: - one for pitch, one for rotation, one for shooting (wheels)


     */

    @Override
    public void init() {
        drivetrain.initDrive(hardwareMap);
        ILC.initILC(hardwareMap, telemetry);

    }

    @Override
    public void loop() {
        drivetrain.driveMecanum(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad2.dpad_left) {
            ILC.lateralTurretManualMove("left");
        }
        if (gamepad2.dpad_right) {
            ILC.lateralTurretManualMove("right");
        }
        if (gamepad2.dpad_up) {
            ILC.verticalTurretManualMove("up");
        }
        if (gamepad2.dpad_down) {
            ILC.verticalTurretManualMove("down");
        }

        if (gamepad2.a) {
            ILC.shootingOn();
        }
        if (gamepad2.b) {
            ILC.shootingOff();
        }

        if (gamepad2.left_bumper) {
            ILC.sorterOn();
        }
        if (gamepad2.right_bumper) {
            ILC.sorterOff();
        }

        if (gamepad2.x) {
            ILC.transferOn();
        }
        if (gamepad2.y) {
            ILC.transferOff();
        }

        if (gamepad1.left_bumper) {
            ILC.intakeOn();
        }
        if (gamepad1.right_bumper) {
            ILC.intakeOff();
        }

        ILC.rateLimit();

        // Get blocks and husky lens data
        HuskyLens.Block[] blocks = ILC.huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);

        while (motifCode == MotifCode.NONE) {
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                if (blocks[i].id == 1) {
                    MotifCode motifCode = MotifCode.TAG_ID_1_GPP;
                    gamepad1.rumble(500);
                    gamepad2.rumble(500);
                } else if (blocks[i].id == 2) {
                    MotifCode motifCode = MotifCode.TAG_ID_2_PGP;
                    gamepad1.rumble(500);
                    gamepad2.rumble(500);
                } else if (blocks[i].id == 3) {
                    MotifCode motifCode = MotifCode.TAG_ID_3_PPG;
                    gamepad1.rumble(500);
                    gamepad2.rumble(500);
                }
            }
        }

        if (motifCode == MotifCode.TAG_ID_1_GPP) {
            switch(motif1) {
                case G:
                    if (ILC.detectGreen()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            ILC.leverOff();
                            ILC.leverOn();
                            ILC.leverOff();
                            ILC.transferOn();
                            motif1 = Motif1GPP.P1;
                        }
                    }
                    break;
                case P1:
                    if (ILC.detectPurple()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            ILC.leverOff();
                            ILC.leverOn();
                            ILC.leverOff();
                            ILC.transferOn();
                            motif1 = Motif1GPP.P2;
                        }
                    }
                    break;
                case P2:
                    if (ILC.detectPurple()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            ILC.leverOff();
                            ILC.leverOn();
                            ILC.leverOff();
                            ILC.transferOn();
                            motif1 = Motif1GPP.G;
                        }
                    }
                    break;
            }
        } else if (motifCode == MotifCode.TAG_ID_2_PGP) {
            switch(motif2) {
                case P1:
                    if (ILC.detectPurple()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            ILC.leverOff();
                            ILC.leverOn();
                            ILC.leverOff();
                            ILC.transferOn();
                            motif2 = Motif2PGP.G;
                        }
                    }
                    break;
                case G:
                    if (ILC.detectGreen()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            ILC.leverOff();
                            ILC.leverOn();
                            ILC.leverOff();
                            ILC.transferOn();
                            motif2 = Motif2PGP.P2;
                        }
                    }
                    break;
                case P2:
                    if (ILC.detectPurple()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            ILC.leverOff();
                            ILC.leverOn();
                            ILC.leverOff();
                            ILC.transferOn();
                            motif2 = Motif2PGP.P1;
                        }
                    }
                    break;
            }
        } else if (motifCode == MotifCode.TAG_ID_3_PPG) {
            switch(motif3) {
                case P1:
                    if (ILC.detectPurple()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            ILC.leverOff();
                            ILC.leverOn();
                            ILC.leverOff();
                            ILC.transferOn();
                            motif3 = Motif3PPG.P2;
                        }
                    }
                    break;
                case P2:
                    if (ILC.detectPurple()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            ILC.leverOff();
                            ILC.leverOn();
                            ILC.leverOff();
                            ILC.transferOn();
                            motif3 = Motif3PPG.G;
                        }
                    }
                    break;
                case G:
                    if (ILC.detectGreen()) {
                        timer.reset();
                        if (timer.milliseconds() == transferElapseTime) {

                            ILC.leverOff();
                            ILC.leverOn();
                            ILC.leverOff();
                            ILC.transferOn();
                            motif3 = Motif3PPG.P1;
                        }
                    }
                    break;
                default:
                    motif3 = Motif3PPG.P1;
            }

        }



        telemetry.update();
    }
}
