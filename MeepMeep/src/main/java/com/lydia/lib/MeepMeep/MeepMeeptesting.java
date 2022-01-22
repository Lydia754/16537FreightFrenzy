package com.lydia.lib.MeepMeep;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class MeepMeeptesting {
        public static void main(String[] args) {
           int meepmap = 800;
           MeepMeep meepMeep = new MeepMeep(meepmap);
           RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                   //robot is 12 in wide
                    .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 8.75)
                    .followTrajectorySequence(drive ->
                            drive.trajectorySequenceBuilder(new Pose2d(8, 64, Math.toRadians(-90)))
                                    /*.forward(30)
                                    .turn(Math.toRadians(90))
                                    .forward(30)
                                    .turn(Math.toRadians(90))
                                    .forward(30)
                                    .turn(Math.toRadians(90))
                                    .forward(30)
                                    .turn(Math.toRadians(90))*/
                                    //.forward(50)
                                   // .back(50)
                                   // .splineTo(new Vector2d(-60,-50) ,Math.toRadians(90))
                                            //new Pose2d(-60,-60, Math.toRadians(90)))
                                    //.forward(20)
                                    .splineTo(new Vector2d(-10, 42),Math.toRadians(-90))
                                    .build()
                    );

            meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
    }
