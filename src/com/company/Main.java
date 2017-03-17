package com.company;

import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;

public class Main {

    public static void main(String[] args) {
        EV3 ev3 = (EV3) BrickFinder.getLocal();

        Catty catty = new Catty(ev3);
        Catty.isTesting = false;
        catty.SSOnRouter = true;
        if (!catty.waitForCommands()) {
            Sound.buzz();
            Sound.beep();
            Sound.buzz();
        }
        Button.ENTER.waitForPress();
        catty.exit();
    }
}
