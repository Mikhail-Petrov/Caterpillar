package com.company;

import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

import java.io.*;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;

/**
 * Created by user on 01.02.16.
 */
class Catty {
    //    Название файла, куда записываются данные.
    private final String mainDataFile = "catData.txt";
    //    Отделитель между командами для записи
    private final String comSeparator = "\n---\n";
    //    Отделитель между командой и временем для записи
    private final String timeSeparator = "\t---\t";
    //    Номер порта, по которому подключаться
    private final int port = 5555;
    //    Номер порта для основного робота
    private final int mainPort = 5555;
    //    Хост, к которому подключаться
    private final String host = "192.168.1.116";
//    private final String host = "192.168.1.134";
    //    Характеристики робота: углы, на которые поднимается/"подтягивается" передний блок, скорости движения/опускания.
    private final int risingAngle = -120;
    private final int shrinkingAngle = 150;
    private final int lowerAngle = 30;
    private float shrinkingCoef = 0;
    private final float badCoef = 0.5f;
    private final int movingSpeed = 150;
    private final int movingSpeed2 = 100;
    private final int risingSpeed = 450;
    private final int risingExploringSpeed = 50;
    //    Допустимая относительная погрешность при поднятии переднего блока
    private final float normalError = 0.1f;
    //    Оптимальная дистанция до препятствия, с которого можно начинать движение.
    private final float optimalDistance = 0.143f;
    //    Минимальная дистанция до препятствия, на которой необходимо остановиться.
    private final float stopDistance = 0.12f;
    //    Коэффициент для перевода из расстояния, определённого датчиком, до расстояния, которое необходимо проехать.
    private final float rotateToDistance = 1400;
    //     Максимальное расстояние до препятствия при измерении его высоты
    private final float maxDistance = 0.2f;
    //    Минимальная значимая разница высоты препятствия
    private final int deltaH = 100;

    private final int deltaI = 20;
    //    Коэффициент для перевода из градусов в количество оборотов для поворота
    final double angleToRotate = 880.0 / 360.0;

    //    Условные обозначения для вставки и удаления из SS
    private final String removeCommand = "rem";
    private final String insertCommand = "ins";
    private final String obstacleInfoPredicate = "obstacleInfo";
    //    Команды для робота/блоков.
    private final String commandForward = "moveForward";
    private final String commandBack = "moveBack";
    private final String commandLeft = "turnLeft";
    private final String commandRight = "turnRight";
    private final String commandTurn = "turnTurn";
    private final String commandStop = "stop";
    private final String commandExit = "exit";
    private final String commandShrink = "shrink";
    private final String commandLower = "lower";
    private final String commandRise = "rise";
    private final String commandClimb = "acrossObstacle";
    private final String commandExploreObst = "exploreObstacle";
    //    Команды для взаимодействия с роботами
    private final String readyCom = "ready";
    private final String distanceCom = "dist";
    //    Данные, заносимые в случае каких-либо событий
    private final String eventPredicate = "event";
    private final String stopEvent = "stopped";
    private final String riseSuccessEvent = "rose";
    private final String lowerSuccessEvent = "lowered";
    //    Команды для записи
    private final String startRecordingSubject = "startRecord";
    private final String finishRecordingSubject = "stopRecord";
    private final String repeatRecordingSubject = "repeatRecord";
    //    Названия объектов робота, заносимые в SS
//    private final String backMotorName = "backBlock";
//    private final String middleMotorName = "middleBlock";
//    private final String forwardMotorName = "frontBlock";
    private final String backMotorName = "block0";
    private final String middleMotorName = "block1";
    private final String forwardMotorName = "block2";
    private final String robotName = "robot";
    private int maxTachoDiff = 15;

    private final String obstBig = "commands for 346-90";
    private final String obstSmall = "commands for 143-335";

    /**
     * Список ультразвуковых датчиков, установленных на роботе.
     */
    private enum USSensor {
        //        forward,
        middle,
        back
    }

    //    Данные, которые необходимо занести в файл.
    private String dataForFile;
    //    Идёт ли тестирование.
    static boolean isTesting;
    //    Определяет, по какому адресу необходимо подключаться к SS: к роутеру или к компьютеру.
    boolean SSOnRouter;

    private Thread thread;

    //    Основной робот или дополнительный
    private boolean isMainRobot;
    //    Едет робот вперёд или назад - для правильной остановки.
    private boolean isGoingForward;
    //    Поднят ли передний блок
    private boolean isRose;

    // Текущее значение TachoCount для моторов
    ArrayList<Integer> rotates;

    //    Замеренные высоты препятствия
    ArrayList<Integer> heights = new ArrayList<Integer>();
    //    Интервалы времени, когда препятствия были обнаружены
    ArrayList<Long> times = new ArrayList<Long>();

    private ArrayList<String> recordNames;

    //    Не используется, но нужна для корректной роботы.
    private EV3 brick;
    //    Моторы.
    private EV3LargeRegulatedMotor middleMotor;
    private EV3LargeRegulatedMotor mainMotor;
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private EV3LargeRegulatedMotor forwardMotor;
    private EV3LargeRegulatedMotor backMotor;
    //    Ультразвуковые датчики и объекты для работы с ними.
    private EV3UltrasonicSensor ultrasonicSensor;
    private SampleProvider sampleProvider;
    private float[] sample;
    private EV3UltrasonicSensor ultrasonicSensorMiddle;
    private SampleProvider sampleProviderMiddle;
    private float[] sampleMiddle;
    //    Взаимодействие с SS.
//    private SmartSpaceKPI smartSpaceKPI;
    //    Взаимодействие с сервером
    private DataOutputStream outputStream;
    private int outputPort;
    private BufferedReader in;


    //    Конструктор
    Catty(EV3 brick) {
        this.brick = brick;

        isTesting = false;
        if (isTesting) {
            test();
            exit();
        }

//        Получить номер порта от сервера, чтобы узнать, основной это робот или дополнительный
        ServerSocket serverSocket = null;
        try {
            serverSocket = new ServerSocket(port);
            LCD.drawString("I am waiting!", 0, 0);
            Sound.beep();
            Socket socket = serverSocket.accept();
            in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            String portNumber = in.readLine();
            LCD.drawString("My port is " + portNumber, 0, 1);
            outputPort = (int) safeParseInt(portNumber);
            isMainRobot = outputPort == mainPort;
            LCD.drawString(String.format("I am %s robot.", isMainRobot ? "main" : "additional"), 0, 2);
        } catch (IOException e) {
            e.printStackTrace();
            exit();
        }

        LCD.drawString("I am starting!", 0, 0);

        rotates = new ArrayList<Integer>();
        if (isMainRobot)
            mainRobotConstructor();
        else
            dopRobotConstructor();

        dataForFile = "";

        isGoingForward = true;
        isRose = false;

        recordNames = new ArrayList<String>();

//        Очистить файл
        getInfoIntoFile(mainDataFile, false);


        Socket outSocket;
        try {
            outSocket = new Socket(host, outputPort);
            outputStream = new DataOutputStream(outSocket.getOutputStream());
            outputStream.writeBytes(readyCom + "\n");
        } catch (SocketException ignored) {
        } catch (IOException e) {
            e.printStackTrace();
        }
        LCD.clear();
        LCD.drawString("I am ready!", 0, 0);
        Sound.beep();
        Sound.beep();
    }

    private void test1() {
        isMainRobot = false;
        dopRobotConstructor();
        LCD.clear();
        dataForFile = "";


        move(true, true, true, true);

        thread = new Thread(new Runnable() {
            @Override
            public void run() {
                float distance;
                while (leftMotor.isMoving()) {
                    mainMotor.setSpeed(risingExploringSpeed);
                    long startTime = System.currentTimeMillis();
                    distance = 0;
                    mainMotor.rotate(risingAngle, true);
                    while (distance < maxDistance) {
                        distance = getDistance(USSensor.middle);
                        dataForFile += String.format("%f4\n", distance);
                    }
//                    heights.add(System.currentTimeMillis() - startTime);
                    mainMotor.stop();
                    mainMotor.setSpeed(risingExploringSpeed * 10);
                    mainMotor.rotate(-mainMotor.getTachoCount());
                }
            }
        });
        thread.start();

        boolean isExplored = false, isEncountered = false;
        float dist;
        long foundTime = 0;
        String info = "", analysis = "";
        while (!isExplored) {
            dist = getDistance(USSensor.back);
            info += String.format("%f3\n", dist);
            if (dist < maxDistance) {
                if (!isEncountered) {
                    isEncountered = true;
                    foundTime = System.currentTimeMillis();
                }
            } else {
                if (isEncountered) {
                    isExplored = true;
                    long endTime = System.currentTimeMillis();
                    analysis = String.format("%d\n%d\n%d\n", foundTime, endTime, endTime - foundTime);
                }
            }
        }
        leftMotor.stop();
        rightMotor.stop();
        getInfoIntoFile(mainDataFile, false);
        dataForFile = info;
        getInfoIntoFile("info.txt", false);
        for (long height : heights)
            analysis += String.format("\n%d", height);
        dataForFile = analysis;
        getInfoIntoFile("analysis.txt", false);
    }

    private void test2() {
        isMainRobot = false;
        dopRobotConstructor();
        LCD.clear();
        dataForFile = "";


        move(true, true, true, true);
        boolean isExplored = false, isEncountered = false;
        float dist;
        long foundTime = 0;
        String info = "", analysis = "";
        while (!isExplored) {
            dist = getDistance(USSensor.middle);
            if (dist > maxDistance)
                dist = getDistance(USSensor.middle);
            info += String.format("%f3\n", dist);
            if (dist < maxDistance) {
                if (!isEncountered) {
                    isEncountered = true;
                    foundTime = System.currentTimeMillis();
                }

//                leftMotor.stop(true);
//                rightMotor.stop();

                mainMotor.setSpeed(risingExploringSpeed);
                long startTime = System.currentTimeMillis();
                mainMotor.backward();
                while (dist < maxDistance) {
                    dist = getDistance(USSensor.middle);
                    dataForFile += String.format("%f4\n", dist);
                }
//                heights.add(System.currentTimeMillis() - startTime);
                mainMotor.stop();
                mainMotor.setSpeed(risingExploringSpeed * 10);
                mainMotor.rotate(-mainMotor.getTachoCount(), false);

//                leftMotor.rotate(50, true);
//                rightMotor.rotate(50);
            } else {
                if (isEncountered) {
                    isExplored = true;
                    long endTime = System.currentTimeMillis();
                    analysis = String.format("%d\n%d\n%d\n", foundTime, endTime, endTime - foundTime);
                }
            }
        }
        leftMotor.stop(true);
        rightMotor.stop();
        getInfoIntoFile(mainDataFile, false);
        dataForFile = info;
        getInfoIntoFile("info.txt", false);
        for (long height : heights)
            analysis += String.format("\n%d", height);
        dataForFile = analysis;
        getInfoIntoFile("analysis.txt", false);
    }

    private void test() {
        isMainRobot = false;
        dopRobotConstructor();
        LCD.clear();
        dataForFile = "";

        float dist;
        String analysis = "";

//        обнаружить препятствие
        dist = getDistance(USSensor.middle);
        if (dist > maxDistance || dist < 0) {
            move(true, true, true, true);
            while (dist > maxDistance || dist < 0) {
                dist = getDistance(USSensor.middle);
                if (dist < maxDistance)
                    dist = getDistance(USSensor.middle);
            }
            leftMotor.stop(true);
            rightMotor.stop();
        }

    }

    private void exploreObstacle() {
        mainMotor.flt();
//        обнаружить препятствие
        float dist = getDistance(USSensor.middle);
        if (dist > maxDistance || dist < 0) {
            move(true, true, true, true);
            while (dist > maxDistance || dist < 0) {
                dist = getDistance(USSensor.middle);
                if (dist < maxDistance)
                    dist = getDistance(USSensor.middle);
            }
            stop2();
        }

        dist = getDistance(USSensor.middle);
        String analysis = "";
//        измерить высоту
        int prevTC = mainMotor.getTachoCount();
        mainMotor.setSpeed(risingExploringSpeed / 2);
        mainMotor.backward();
        while (dist < maxDistance) {
            dist = getDistance(USSensor.middle);
            if (dist >= maxDistance)
                dist = getDistance(USSensor.middle);
            dataForFile += String.format("%4f\n", dist);
        }
        mainMotor.stop();
        int height = prevTC - mainMotor.getTachoCount();
        prevTC = mainMotor.getTachoCount();
        heights.clear();
        times.clear();
        heights.add(height);
        times.add(System.currentTimeMillis());
        getInfoIntoFile(mainDataFile, false);

//        измерять высоты
        mainMotor.setSpeed(risingExploringSpeed);
        int exploringSpeed = (int) (movingSpeed2 / 1.5);
        setMovingSpeed(exploringSpeed);
        int startPosition = getAvTacho(true), curPosition = getAvTacho(true) - startPosition;
        boolean isDown;
        move(true, true, true, true);
        int maxH = 340;
        while (height > 0 || curPosition < deltaI) {
//            если видим препятствие, то поднимать, если нет, то опускать
//            при потере из вида фиксировать высоту
            isDown = dist > maxDistance || height >= maxH;
//            LCD.drawString(String.format("h: %d", height), 1, 3);
//            LCD.drawString(String.format("d: %f", dist), 1, 4);

            boolean stopped = false;
            if (isDown)
                mainMotor.forward();
            else
                mainMotor.backward();
            int changePosition = getAvTacho(false);
            while ((dist > maxDistance || height >= maxH) && isDown || dist < maxDistance && height < maxH && !isDown) {
                dist = getDistance(USSensor.middle);
                dataForFile += String.format("%4f\n", dist);
                height += prevTC - mainMotor.getTachoCount();
//                LCD.drawString(String.format("h: %d", height), 1, 3);
//                LCD.drawString(String.format("d: %f", dist), 1, 4);
                prevTC = mainMotor.getTachoCount();
                curPosition = getAvTacho(stopped) - startPosition;
                if (curPosition > heights.size() * deltaI) {
                    heights.add(height);
                    times.add(System.currentTimeMillis());
                }
                if (getAvTacho(stopped) - changePosition > deltaI) {
                    stop2();
                    stopped = true;
                }
                if (isDown && height <= 0)
                    break;
                if (dist <= maxDistance)
                    dist = getDistance(USSensor.middle);
            }
            if (stopped && height > 0)
                move(true, true, true, true);
            mainMotor.stop();
                /*if (height > 0) {
                    heights.add(height);
                    times.add(System.currentTimeMillis());
                }*/
        }
        maxTachoDiff = 5;
        getAvTacho(true);
        maxTachoDiff = 15;

        sendToRetranslater(String.format("%s\t%d\n", distanceCom, stop(true, true, true)));
        getInfoIntoFile(mainDataFile, false);
        for (long h : heights)
            analysis += String.format("%d,", h);
        dataForFile = analysis;
        getInfoIntoFile("analysis.txt", false);

//        Анализ времени изменения высоты
        if (heights.isEmpty())
            return;
        analysis = heights.get(0).toString();
        if (heights.size() > 1) {
            dataForFile = "";
            for (int i = 1; i < heights.size(); i++) {
                analysis += String.format("; %d", heights.get(i));
                long time = times.get(i) - times.get(i - 1);
                int delta = heights.get(i) - heights.get(i - 1);
                dataForFile += String.format("%d\t%d\n", time, delta);
                /*if (Math.abs(delta) > deltaH)
                    analysis += String.format("; %d: %d", (times.get(i) - times.get(0)) * exploringSpeed, delta);*/
            }
            getInfoIntoFile("times.txt", false);
        }

        setMovingSpeed(movingSpeed2);

//        writeToSS(null, obstacleInfoPredicate, analysis);
        writeToSS(analysis, "task", obstacleInfoPredicate);
//        sendToRetranslater("test" + "\t" + analysis + "\n");
        LCD.drawString(String.format("%s\t%s\t%s\t%s", analysis, "task", obstacleInfoPredicate, insertCommand), 0, 0);
        dataForFile = analysis;
        getInfoIntoFile("analysis.txt", false);
    }

    private void stop2() {
        leftMotor.stop(true);
        rightMotor.stop();
//        leftMotor.flt();
//        rightMotor.flt();
    }

    private int getAvTacho(boolean stopped) {
        int leftRes = leftMotor.getTachoCount(), rightRes = rightMotor.getTachoCount();
        int res = (leftRes + rightRes) / 2;
//        LCD.drawString(String.format("l: %d", leftRes), 1, 1);
//        LCD.drawString(String.format("r: %d", rightRes), 1, 2);
        if (Math.abs(rightRes - leftRes) > maxTachoDiff) {
            if (!stopped)
                stop2();
            leftMotor.rotate(rightRes - leftRes);
            if (!stopped)
                move(true, true, true, true);
        }
        return res;
    }

    /**
     * Подключить моторы и датчики у главного робота
     */
    private void mainRobotConstructor() {
        middleMotor = new EV3LargeRegulatedMotor(MotorPort.A);
        mainMotor = new EV3LargeRegulatedMotor(MotorPort.B);
        forwardMotor = new EV3LargeRegulatedMotor(MotorPort.C);
        backMotor = new EV3LargeRegulatedMotor(MotorPort.D);
        mainMotor.setSpeed(risingSpeed);
        setMovingSpeed(movingSpeed);
        rotates.add(forwardMotor.getTachoCount());
        rotates.add(middleMotor.getTachoCount());
        rotates.add(backMotor.getTachoCount());

        boolean successful = false;
        while (!successful)
            try {
                ultrasonicSensorMiddle = new EV3UltrasonicSensor(SensorPort.S2);
                sampleProviderMiddle = ultrasonicSensorMiddle.getDistanceMode();
                sampleMiddle = new float[sampleProviderMiddle.sampleSize()];
                successful = true;
            } catch (Exception e) {
                Sound.buzz();
                Sound.buzz();
            }
    }

    /**
     * Подключить моторы и датчики у дополнительного робота
     */
    private void dopRobotConstructor() {
        mainMotor = new EV3LargeRegulatedMotor(MotorPort.B);
        leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
        rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
        mainMotor.setSpeed(risingSpeed);
        setMovingSpeed(movingSpeed2);
        rotates.add(leftMotor.getTachoCount());
        rotates.add(rightMotor.getTachoCount());

        boolean successful = false;
        while (!successful)
            try {
//                ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S3);
//                sampleProvider = ultrasonicSensor.getDistanceMode();
//                sample = new float[sampleProvider.sampleSize()];
                successful = true;
            } catch (Exception e) {
                Sound.buzz();
                Sound.buzz();
                Sound.buzz();
            }
        successful = false;
        while (!successful)
            try {
                ultrasonicSensorMiddle = new EV3UltrasonicSensor(SensorPort.S2);
                sampleProviderMiddle = ultrasonicSensorMiddle.getDistanceMode();
                sampleMiddle = new float[sampleProviderMiddle.sampleSize()];
                successful = true;
            } catch (Exception e) {
                Sound.buzz();
                Sound.buzz();
            }
    }

    /**
     * Закончить выполнение.
     */
    void exit() {
//        leave();
        Runtime.getRuntime().exit(0);
    }

    /**
     * Отправляет на сервер команду удаления и вставки заданной тройки.
     *
     * @param subject   субъект тройки для вставки
     * @param predicate предикат тройки для вставки
     * @param object    объект тройки для вставки
     */
    private void writeToSS(String subject, String predicate, String object) {
        if (outputStream != null)
            try {
//                outputStream.writeBytes(String.format("%s\t%s\t%s\t%s\n", subject, predicate, object, removeCommand));
                outputStream.writeBytes(String.format("%s\t%s\t%s\t%s\n", subject, predicate, object, insertCommand));
            } catch (IOException e) {
                e.printStackTrace();
            }
    }

    /**
     * Двигаться вперёд или назад, пока на расстоянии, меньше заданного, не появится объект.
     *
     * @param goalDistance целевое расстояние до объекта
     * @param goForward    если true, то двигаться вперёд, иначе назад.
     */
    private void goToObstacle(float goalDistance, boolean goForward) {
        rise();
//        float distance = getDistance(USSensor.forward);
        float distance = getDistance(USSensor.middle);
        if (distance > goalDistance)
            move(true, true, true, goForward);
        while (distance > goalDistance)
//            distance = getDistance(USSensor.forward);
            distance = getDistance(USSensor.middle);
        double dist = stop(true, true, true);
//        lower();
        /*try {
//            smartSpaceKPI.insert(new SmartSpaceTriplet(stopEvent, eventPredicate, "null"));
            smartSpaceKPI.remove(new SmartSpaceTriplet(robotName, eventPredicate, stopEvent));
            smartSpaceKPI.insert(new SmartSpaceTriplet(robotName, eventPredicate, stopEvent));
        } catch (SmartSpaceException e) {
            e.printStackTrace();
        }*/
        sendToRetranslater(String.format("%s\t%d\n", stopEvent, (int) dist));
//        writeToSS(robotName, eventPredicate, stopEvent);
    }

    private void rise2() {
        mainMotor.rotate(risingAngle);
    }

    /**
     * Поднять передний модуль
     */
    private void rise() {
//        if (isRose)
//            return;
        backMotor.stop();
        middleMotor.stop();
        mainMotor.resetTachoCount();
        mainMotor.rotate(risingAngle, false);
//        Проверить, удалось ли поднять.
        int tachoCount = mainMotor.getTachoCount() + 1;
        if (Math.abs(risingAngle / tachoCount - 1) <= normalError) {
//            writeToSS(forwardMotorName, eventPredicate, riseSuccessEvent);
            /*
//                Если удалось поднять, отправить инфу в SS.
            if (smartSpaceKPI != null)
                try {
//                    smartSpaceKPI.insert(new SmartSpaceTriplet(riseSuccessEvent, eventPredicate, String.format("%d", tachoCount)));
                    smartSpaceKPI.remove(new SmartSpaceTriplet(forwardMotorName, eventPredicate, riseSuccessEvent));
                    smartSpaceKPI.insert(new SmartSpaceTriplet(forwardMotorName, eventPredicate, riseSuccessEvent));
                } catch (SmartSpaceException e) {
                    e.printStackTrace();
                }*/

            shrinkingCoef = 1;
            isRose = true;
        } else {
//                Если не удалось поднять, отправить инфу в файл и изменить коэффициент.
//            if (smartSpaceKPI == null) {
            if (outputStream == null) {
                dataForFile = String.format("Can't rise. Tacho count: %d", tachoCount);
                getInfoIntoFile(mainDataFile);
            }
            shrinkingCoef += Math.abs(tachoCount / risingAngle);
        }
    }

    /**
     * Опустить передний модуль
     */
    private void lower() {
//        if (isRose)
        mainMotor.rotate(lowerAngle);
        mainMotor.flt(true);
        shrinkingCoef = 0;
        isRose = false;
//        writeToSS(forwardMotorName, eventPredicate, lowerSuccessEvent);
    }

    /**
     * "Подтянуть" задние модули.
     */
    private void shrink() {
        forwardMotor.stop();
        if (shrinkingCoef == 0)
            mainMotor.rotate(shrinkingAngle);
        else
            mainMotor.rotate((int) (-risingAngle * shrinkingCoef));
//        mainMotor.rotate((int) (shrinkingAngle * shrinkingCoef));
        shrinkingCoef = 0;
        isRose = false;
        writeToSS(forwardMotorName, eventPredicate, lowerSuccessEvent);
    }

    /**
     * Записывать информацию о ходе тестирования SS.
     *
     * @param info информация, которую необходимо записать в файл.
     */
    private void writeInfoForTesting(String info) {
        if (isTesting) {
            dataForFile = info;
            getInfoIntoFile(mainDataFile);
        }
    }

    /**
     * Взобраться на препятствие.
     */
    private void climb() {
//        Оценить расстояние до объекта
//        rise();
//        Подъехать
        /*float dist = getDistance(USSensor.middle);
        if (dist > stopDistance || dist < 0) {
//            move(true, true, true, true);
            move(150, true);
            while (dist > stopDistance || dist < 0 || forwardMotor.isMoving()) {
                dist = getDistance(USSensor.middle);
                if (dist < stopDistance)
                    dist = getDistance(USSensor.middle);
            }
            stop(true, true, true);
        }*/
//        Опустить передний блок
        move(200);
        lower();
//        mainMotor.flt();
//        Вскарабкаться, пока не устаканится расстояние до пола
//        /*move(true, true, true, true);
//        float min = 0.1f, max = 0.2f, cur = 0, sum = 0, avg = 0, prevAvg = 0, dif = 0;
//        float[] last = new float[50];
//        int curI = 0;
//        boolean itsTime = false;
//        dataForFile = "";
//        for (int i = 0; forwardMotor.isMoving(); i++) {
//            cur = getDistance(USSensor.middle);
//            if (cur == Float.POSITIVE_INFINITY)
//                cur = 2;
//            dataForFile += String.format("%f\t", cur);
//            sum += cur;
//            if (i < 50)
//                last[i] = cur;
//            else {
//                curI = i % 50;
//                sum -= last[curI];
//                last[curI] = cur;
//                avg = sum / (float) last.length;
//                dif = Math.abs(1f - avg / prevAvg);
//                if (itsTime) {
//                    if (avg > min && avg < max && dif < 0.01f)
//                        stop(true, true, true);
//                } else if (avg > 2 * max)
//                    itsTime = true;
//                prevAvg = avg;
//            }
//            writeInfoForTesting(String.format(Locale.FRANCE, "%f\t%f\t%d\n", cur, avg, itsTime ? 1 : 0));
//        }
//        stop(true, true, true);
//        getInfoIntoFile("distanceInfo");*/
        shrink();
        move(1200);
//        backMotor.flt();
        mainMotor.flt();

//        middleMotor.rotate(200, false);
//        backMotor.rotate(100);
//        move(800);
    }

    private void climbSmall() {
//        первая ступень
        move(250);
        lower();
        shrink();
        move(300);
//        вторая ступень
        mainMotor.flt();
        forwardMotor.rotate(50);
        mainMotor.rotate(risingAngle / 2);
        move(200);
        lower();
        shrink();
        move(700);
    }

    /**
     * Получить показания с ультразвукового датчика.
     *
     * @param sensor указывает, с какого датчика получать показания
     * @return расстояние до ближайшего объекта.
     */
    private float getDistance(USSensor sensor) {
        /*if (sensor == USSensor.forward) {

        } else */
        if (sensor == USSensor.middle) {
            sampleProviderMiddle.fetchSample(sampleMiddle, 0);
            if (sampleMiddle[0] == Float.POSITIVE_INFINITY)
//                sampleMiddle[0] = -0.1f;
                sampleMiddle[0] = 3f;
            float dist = sampleMiddle[0];
            sampleProviderMiddle.fetchSample(sampleMiddle, 0);
            if (sampleMiddle[0] == Float.POSITIVE_INFINITY)
//                sampleMiddle[0] = -0.1f;
                sampleMiddle[0] = 3f;
            LCD.clearDisplay();
            LCD.drawString(String.format("%f", (sampleMiddle[0] + dist) / 2f), 1, 1);
            return (sampleMiddle[0] + dist) / 2f;
        } else if (sensor == USSensor.back) {
            sampleProvider.fetchSample(sample, 0);
            LCD.clearDisplay();
            LCD.drawString(String.format("%f", sampleMiddle[0]), 1, 1);
            return sample[0];
        }
        return Float.POSITIVE_INFINITY;
    }

    /**
     * Воспроизвести записанную ранее последовательность действий.
     *
     * @param recordName название последовательности, которую необходимо воспроизвести.
     */
    private void repeatRecord(String recordName) {
        String data = getDataFromFile(recordName);
        String[] commands = data.split(comSeparator);
        long time = -1;
        for (String command : commands) {
            long curTime = safeParseInt(command.split(timeSeparator)[0]);
            if (time >= 0) {
                try {
                    Thread.sleep(curTime - time);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            time = curTime;
            executeCommand(command.split(timeSeparator)[1]);
        }
    }

    /**
     * Прочитать данные из файла.
     *
     * @param filename имя файла, из которого необходимо прочитать данные
     * @return данные, содержащиеся в файле в виде строки.
     */
    private String getDataFromFile(String filename) {
        String data = null;
        try {
            File dataFile = new File(filename);
            BufferedInputStream file = new BufferedInputStream(new FileInputStream(dataFile));
            byte[] b = new byte[(int) dataFile.length()];
            file.read(b);
            data = new String(b);
            file.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return data;
    }

    /**
     * Подключиться к серверу и обрабатывать команды, которые поступают оттуда.
     *
     * @return false, если подключиться не удалось.
     */
    boolean waitForCommands() {
        try {
            while (true) {
                String data = in.readLine();
                String[] commands = data.split("\n");
                for (String command : commands)
                    executeCommand(command);
                if (!recordNames.isEmpty()) {
                    Date curDate = new Date();
                    long curTime = curDate.getTime();
                    for (String recordName : recordNames) {
                        dataForFile = String.format("%d%s%s%s", curTime, timeSeparator, data, comSeparator);
                        getInfoIntoFile(recordName);
                    }
                }
                Sound.twoBeeps();
            }
        } catch (SocketException e) {
            Sound.beep();
            exit();
            return true;
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }
    }

    /**
     * Установить значение скорости для ведущих колёс.
     *
     * @param speed значение скорости.
     */
    private void setMovingSpeed(int speed) {
        if (isMainRobot) {
            middleMotor.setSpeed(speed);
            forwardMotor.setSpeed(speed);
            backMotor.setSpeed(speed);
        } else {
            leftMotor.setSpeed(speed);
            rightMotor.setSpeed(speed);
        }
    }

    /**
     * Остановить движение заданных моторов.
     *
     * @param forward если true, то передняя пара колёс будет остановлена
     * @param middle  если true, то средняя пара колёс будет остановлена
     * @param back    если true, то задняя пара колёс будет остановлена.
     */
    private int stop(boolean forward, boolean middle, boolean back) {
        if (isMainRobot)
//        Останавливать сначала задний (по ходу движения) мотор, потом средний, потом передний.
            if (isGoingForward) {
                if (back)
                    if (backMotor.isMoving())
                        backMotor.stop(true);
                if (middle)
                    if (middleMotor.isMoving())
                        middleMotor.stop(true);
                if (forward)
                    if (forwardMotor.isMoving())
                        forwardMotor.stop();
            } else {
                if (forward)
                    if (backMotor.isMoving())
                        forwardMotor.stop(true);
                if (middle)
                    if (middleMotor.isMoving())
                        middleMotor.stop(true);
                if (back)
                    if (forwardMotor.isMoving())
                        backMotor.stop();
            }
        else {
            if (leftMotor.isMoving())
                leftMotor.stop(true);
            if (rightMotor.isMoving())
                rightMotor.stop();
        }
        return updateRotates(forward, middle, back);// * (isGoingForward ? 1 : -1);
    }

    /**
     * Двигаться вперёд или назад, пока не придёт команда остановиться.
     *
     * @param forward   если true, то передняя пара колёс будет задействована
     * @param middle    если true, то средняя пара колёс будет задействована
     * @param back      если true, то задняя пара колёс будет задействована
     * @param goForward если true, то двигаться вперёд, иначе назад.
     */
    private void move(boolean forward, boolean middle, boolean back, boolean goForward) {
        isGoingForward = goForward;
        if (goForward) {
            if (isMainRobot) {
                if (forward)
//                forwardMotor.backward();      //Передний мотор поставлен задом наперёд =)
                    forwardMotor.forward();
                if (middle)
                    middleMotor.forward();
                if (back)
//                    backMotor.forward();
                    backMotor.backward();
            } else {
                leftMotor.forward();
                rightMotor.forward();
            }
        } else {
            if (isMainRobot) {
                if (forward)
//                forwardMotor.forward();      //Передний мотор поставлен задом наперёд =)
                    forwardMotor.backward();
                if (middle)
                    middleMotor.backward();
                if (back)
                    backMotor.backward();
            } else {
                leftMotor.backward();
                rightMotor.backward();
            }
        }
    }

    /**
     * Двигаться на заданное расстояние, используя все моторы.
     *
     * @param angle расстояние, на которое необходимо двигаться.
     */
    private void move(int angle) {
        move(angle, false);
    }

    /**
     * Двигаться на заданное расстояние, используя все моторы с возможностью не дожидаться окончания выполнения.
     *
     * @param angle           расстояние, на которое необходимо двигаться.
     * @param immediateReturn если true, то управление возвращается сразу, не дожидаясь окончания движения.
     */
    private void move(int angle, boolean immediateReturn) {
        move(angle, true, true, true, immediateReturn);
    }

    /**
     * Двигаться на заданное расстояние, используя определённые моторы.
     *
     * @param angle   расстояние, на которое необходимо двигаться.
     * @param forward если true, то передняя пара колёс будет задействована
     * @param middle  если true, то средняя пара колёс будет задействована
     * @param back    если true, то задняя пара колёс будет задействована
     */
    private void move(int angle, boolean forward, boolean middle, boolean back) {
        move(angle, forward, middle, back, false);
    }

    /**
     * Двигаться на заданное расстояние, используя определённые моторы с возможностью не дожидаться окончания выполнения.
     * Возвращает пройденное расстояние
     *
     * @param angle           расстояние, на которое необходимо двигаться.
     * @param forward         если true, то передняя пара колёс будет задействована
     * @param middle          если true, то средняя пара колёс будет задействована
     * @param back            если true, то задняя пара колёс будет задействована
     * @param immediateReturn если true, то управление возвращается сразу, не дожидаясь окончания движения.
     */
    private int move(int angle, boolean forward, boolean middle, boolean back, boolean immediateReturn) {
        isGoingForward = angle > 0;
        if (isMainRobot) {
            if (forward)
//            forwardMotor.rotate(-angle, true);      //Передний мотор поставлен задом наперёд =)
                forwardMotor.rotate(angle, true);
            if (middle)
                middleMotor.rotate(angle, true);
            if (back)
//                backMotor.rotate(angle, true);
                backMotor.rotate(-angle, true);
        } else {
            leftMotor.rotate(angle, true);
            rightMotor.rotate(angle, true);
        }
        if (!immediateReturn) {
            if (isTesting)
                while (forward && forwardMotor.isMoving() || middle && middleMotor.isMoving() || back && backMotor.isMoving())
                    writeInfoForTesting(String.format(Locale.FRANCE, "%f\t", getDistance(USSensor.middle)).replace("Infinity", "0"));
            else {
                if (isMainRobot) {
                    if (forward)
                        forwardMotor.waitComplete();
                    if (middle)
                        middleMotor.waitComplete();
                    if (back)
                        backMotor.waitComplete();
                } else {
                    leftMotor.waitComplete();
                    rightMotor.waitComplete();
                }
                return updateRotates(forward, middle, back);
            }
        }
        return 0;
    }

    /**
     * Повернуться на заданный угол по часовой стрелке (только для дополнительного робота)
     *
     * @param angle угол поворота в градусах
     */
    private void turn(int angle) {
        if (isMainRobot)
            return;
        int rotate = (int) (angle * angleToRotate);
        leftMotor.rotate(rotate, true);
        rightMotor.rotate(-rotate, true);
        leftMotor.waitComplete();
        rightMotor.waitComplete();
        updateRotates();
    }

    /**
     * Записать данные, содержащиеся в параметре dataForFile, в файл с указанным именем.
     * Если такой файл уже существует, данные будут записаны в конец файла.
     *
     * @param fileName имя файла, в который будут записаны данные
     */
    private void getInfoIntoFile(String fileName) {
        getInfoIntoFile(fileName, true);
    }

    /**
     * Записать данные, содержащиеся в параметре dataForFile, в файл с указанным именем.
     * Если такой файл уже существует, данные могут быть записаны в конец файла либо вместо существующих.
     *
     * @param fileName имя файла, в который будут записаны данные
     * @param toAdd    если false, данные будут записаны вместо существующих, иначе - в конец файла.
     */
    private void getInfoIntoFile(String fileName, boolean toAdd) {
        if (dataForFile.isEmpty() && toAdd)
            return;
        try {
            BufferedOutputStream file = new BufferedOutputStream(new FileOutputStream(new File(fileName), toAdd));
            try {
                file.write(dataForFile.getBytes());
                dataForFile = "";
            } catch (IOException e) {
                e.printStackTrace();
            }

            file.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Получить число из строки, не получая исключений.
     *
     * @param s строка, в которой число
     * @return число, которое было в строке, либо 0, если распарсить не удалось.
     */
    private long safeParseInt(String s) {
        long value;
        try {
            value = new Long(s);
        } catch (NumberFormatException e) {
            value = 0;
        }
        return value;
    }

    /**
     * Обновляет значения текущего TachoCount для указанных моторов и возвращает пройденную дистанцию.
     *
     * @param forward учитывать ли передний мотор
     * @param middle  учитывать ли средний мотор
     * @param back    учитывать ли задний мотор
     * @return пройденная дистанция
     */
    private int updateRotates(boolean forward, boolean middle, boolean back) {
        int avgDistance = 0;
        int curMotor = 0;
        if (isMainRobot) {
            if (forward) {
                rotates.set(0, forwardMotor.getTachoCount() - rotates.get(0));
                avgDistance += rotates.get(0);
                curMotor++;
            }
            if (middle) {
                rotates.set(1, middleMotor.getTachoCount() - rotates.get(1));
                avgDistance += rotates.get(1);
                curMotor++;
            }
            if (back) {
                rotates.set(2, backMotor.getTachoCount() - rotates.get(2));
                avgDistance += rotates.get(2);
                curMotor++;
            }
            LCD.clear();
            for (int i = 0; i < rotates.size(); i++) {
//                LCD.drawString(String.format("%d", rotates.get(i)), 0, i + 1);
            }
        } else {
            avgDistance += leftMotor.getTachoCount() - rotates.get(0);
            rotates.set(0, leftMotor.getTachoCount());
            avgDistance += rightMotor.getTachoCount() - rotates.get(1);
            rotates.set(1, rightMotor.getTachoCount());
            curMotor = 2;
            LCD.clearDisplay();
            LCD.drawString(String.format("l: %d", rotates.get(0)), 1, 1);
            LCD.drawString(String.format("r: %d", rotates.get(1)), 1, 2);
        }
        if (curMotor > 0)
            avgDistance /= curMotor;
        return avgDistance;
    }

    private double updateRotates() {
        return updateRotates(true, true, true);
    }

    private String getCurrentTime() {
        return new SimpleDateFormat("HH:mm:ss:S", Locale.GERMANY).format(new Date());
    }

    private void sendToRetranslater(String msg) {
        if (!msg.endsWith("\n"))
            msg += "\n";
        try {
            outputStream.writeBytes(msg);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Выполнить команду.
     *
     * @param command - выполняемая команда
     */
    private void executeCommand(String command) {
        int dist;
        final int attrsMinAmount = 3;
        final int delayAttr = 0;
        final int blockAttr = 1;
        final int commAttr = 2;
        String[] comAttrs = command.split("\t");
        if (comAttrs.length < attrsMinAmount)
            return;
        long delay = safeParseInt(comAttrs[delayAttr]);
        if (delay > 0)
            try {
                Thread.sleep(delay);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        String execBlock = comAttrs[blockAttr];
        boolean fBlock = execBlock.equals(forwardMotorName),
                mBlock = execBlock.equals(middleMotorName),
                bBlock = execBlock.equals(backMotorName);
        if (execBlock.isEmpty()) {
            fBlock = true;
            mBlock = true;
            bBlock = true;
        }

        if (comAttrs[commAttr].equals(commandStop)) {
//         Остановка
            dist = stop(fBlock, mBlock, bBlock);
            sendToRetranslater(String.format("%s\t%d\n", distanceCom, dist));
        } else if (comAttrs[commAttr].equals(commandForward) || comAttrs[commAttr].equals(commandBack)) {
//            Движение вперёд или назад
            long distance = 0;
            if (comAttrs.length > attrsMinAmount) {
                distance = safeParseInt(comAttrs[commAttr + 1]);
            }
            if (distance == 0) {
                if (comAttrs[commAttr].equals(commandForward)) {
                    thread = new Thread(new Runnable() {
                        @Override
                        public void run() {
                            goToObstacle(stopDistance, true);
//                            move(true, true, true, true);
                        }
                    });
                    thread.start();
                } else
                    move(fBlock, mBlock, bBlock, false);
            } else {
                if (comAttrs[commAttr].equals(commandBack))
                    distance *= -1;
                dist = move((int) distance, fBlock, mBlock, bBlock, false);
//                Если был указан угол поворота, то указать его в ответе
                String msg = String.format("%s\t%d", distanceCom, dist);
                if (comAttrs.length > attrsMinAmount + 1)
                    msg += "\t" + comAttrs[attrsMinAmount + 1];
                sendToRetranslater(msg + "\n");
            }
        } else if (comAttrs[commAttr].equals(commandLeft)) {
//            Повернуть налево
            turn(-90);
        } else if (comAttrs[commAttr].equals(commandRight)) {
//            Повернуть направо
            turn(90);
        } else if (comAttrs[commAttr].equals(commandTurn)) {
//            Повернуть на заданный угол
            if (comAttrs.length > attrsMinAmount) {
                int angle = (int) safeParseInt(comAttrs[attrsMinAmount]);
                turn(angle);
            }
        } else if (comAttrs[commAttr].equals(commandRise)) {
//            Поднять блок
            if (isMainRobot)
                rise();
            else
                rise2();
        } else if (comAttrs[commAttr].equals(commandLower)) {
//            Отпустить блок
            lower();
        } else if (comAttrs[commAttr].equals(commandShrink)) {
//            Подтянуть блок
            shrink();
        } else if (comAttrs[commAttr].equals(commandExit)) {
//            Выход
            exit();
        } else if (comAttrs[commAttr].equals(commandClimb)) {
//            Преодолеть препятствие
            String obstType = obstBig;
            if (comAttrs.length > attrsMinAmount)
                obstType = comAttrs[attrsMinAmount];
            if (obstType.equals(obstBig))
                climb();
            else if (obstType.equals(obstSmall))
                climbSmall();
        } else if (comAttrs[commAttr].equals(commandExploreObst)) {
//            Исследовать препятствие
            exploreObstacle();
            mainMotor.flt();
        }
        // TODO: имена записей
        /*else if (command.equals(startRecordingSubject)) {
//            Начать запись
            if (recordNames.indexOf(object) == -1)
                recordNames.add(object);
        } else if (command.equals(finishRecordingSubject)) {
//            Закончить запись
            recordNames.remove(object);
        } else if (command.equals(repeatRecordingSubject)) {
//            Воспроизвести запись
            repeatRecord(object);*/
    }

}
