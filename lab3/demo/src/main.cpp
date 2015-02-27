#include "ped_model.h"
#include "MainWindow.h"
#include "ParseScenario.h"

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QApplication>
#include <QTimer>
#include <thread>

#include <unistd.h>

#include "Timer.h"
#include <iostream>
#include <chrono>
#include <ctime>
#include <cstring>
#include <fstream>

#define SIMULATION_STEPS 1000000
#define TICK_LIMIT 1000//0
#define AS_FAST_AS_POSSIBLE 0

int main(int argc, char*argv[]) { 
    Ped::Model model;
    bool timing_mode = 0;
    bool silent = false;
    bool plot = false;
    int i = 1;
    size_t threads = 4;
    QString scenefile = "scenario.xml";
    Ped::IMPLEMENTATION IMP = Ped::IMPLEMENTATION::SEQ;

    while(i < argc) {
        if(argv[i][0] == '-' && argv[i][1] == '-') {
            if(strcmp(&argv[i][2],"timing-mode") == 0) {
                if(!silent) cout << "Timing mode on" << endl;
                timing_mode = true;
	    } else if(strcmp(&argv[i][2],"omp") == 0) {
                IMP = Ped::IMPLEMENTATION::OMP;
	    } else if(strcmp(&argv[i][2], "threads") == 0) {
                i+=1;
                threads = atoi((const char*) argv[i]);
            } else if(strcmp(&argv[i][2], "silent") == 0) {
                silent = true;
            } else if(strcmp(&argv[i][2], "plot") == 0) {
                plot = true;
            } else if(strcmp(&argv[i][2], "rebuild") == 0) {
			  model.enableRebuild();
			} else {
			  cerr << "Unrecognized command: \"" << argv[i] << "\". Ignoring ..." << endl;
	    }
	} else {
            scenefile = argv[i];
	}
        i+=1;
    }
    if(!silent) {
        cout << "Running using ";
        switch(IMP) {
	case Ped::IMPLEMENTATION::OMP:
            cout << "OpenMP implementation (" << threads << " threads)" << endl;
            break;
        default:
            cout << "sequential implementation" << endl;
            break;
        }
    }
    ParseScenario parser(scenefile);
    model.setup(parser.getAgents(),IMP);
    model.setNumThreads(threads);

    QApplication app(argc, argv);
  
    MainWindow mainwindow(model);

    const int delay_ms = 16;
    Timer *timer;
    if(timing_mode) {
        timer = new Timer(model,mainwindow,AS_FAST_AS_POSSIBLE);
        timer->setTickLimit(TICK_LIMIT);
    } else {
        timer = new Timer(model,mainwindow,delay_ms);
        mainwindow.show();
    }
    if(!silent) cout << "Demo setup complete, running ..." << endl;
    int retval = 0;
    std::chrono::time_point<std::chrono::system_clock> start,stop;
    start = std::chrono::system_clock::now();

    if(timing_mode) {
        timer->busyTick();
    } else {
        timer->qtTimerTick();
        retval = app.exec();
    }

    stop = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = stop-start;
    if(!silent) cout << "Time: " << elapsed_seconds.count() << " seconds." << endl;
    
    if(plot) {
      fstream file;
      file.open("./testdata.txt", std::fstream::out | std::fstream::app);
      file << ((IMP == Ped::IMPLEMENTATION::OMP) ? "OpenMP  " : "Sequential") << "\t" << threads << "\t" << elapsed_seconds.count() << endl;
      file.close();
    }

    if(!silent) cout << "Done" << endl;
    delete (timer);
    return retval;
}
