//#include "ped_agent.h"
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



int main(int argc, char*argv[]) { 
  Ped::Model model;
  bool timing_mode = 0;
  bool silent = false;
  bool plot = false;
  int i = 1;
  size_t threads = 4;
  QString scenefile = "scenario.xml";
  Ped::IMPLEMENTATION IMP = Ped::IMPLEMENTATION::SEQ;
  // Argument handling
  while(i < argc)
    {
      if(argv[i][0] == '-' && argv[i][1] == '-')
	  {
		if(strcmp(&argv[i][2],"timing-mode") == 0)
		{
                  if(!silent)
                    cout << "Timing mode on\n";
	      timing_mode = true;
	    }
		else if(strcmp(&argv[i][2], "pthreads") == 0)
		{
		  IMP = Ped::IMPLEMENTATION::PTHREAD;
		}
		else if(strcmp(&argv[i][2], "omp") == 0)
		{
		  IMP = Ped::IMPLEMENTATION::OMP;
		}
		else if(strcmp(&argv[i][2], "threads") == 0)
		{
		  i+=1;
		  threads = atoi((const char*) argv[i]);
		}
		else if(strcmp(&argv[i][2], "silent") == 0)
		{
		  silent = true;
		}
		else if(strcmp(&argv[i][2], "plot") == 0)
		{
		  plot = true;
		}
		else if(strcmp(&argv[i][2], "simd") == 0 || strcmp(&argv[i][2], "vector") == 0)
		{
		  Ped::Tvector::VEC = true;
		}
		else
		{
		  cerr << "Unrecognized command: \"" << argv[i] << "\". Ignoring ..." << endl;
		}
	  }
      else // Assume it is a path to scenefile
	  {
		scenefile = argv[i];
	  }
      i+=1;
    }
  if(!silent)
    {
      std::cout << "Running using ";
      switch(IMP)
        {
	case Ped::IMPLEMENTATION::SEQ:
	  std::cout << "sequential implementation";
	  break;
	case Ped::IMPLEMENTATION::OMP:
	  std::cout << "OpenMP implementation (" << threads << " threads)";
	  break;
	case Ped::IMPLEMENTATION::PTHREAD:
	  std::cout << "Pthread implementation (" << threads << " threads)";
	  break;
	default:
	  break;
        }
	  if(Ped::Tvector::VEC)
		std::cout << " using SIMD operations." << std::endl;
	  else
		std::cout << "." << std::endl;
    }
  ParseScenario parser(scenefile);
  model.setup(parser.getAgents());
  model.setImplementation(IMP);
  model.setNumThreads(threads);

  QApplication app(argc, argv);
  
  MainWindow mainwindow(model);


  

  const int delay_ms = 100;
  Timer *timer;
#define TICK_LIMIT 10000
  #define AS_FAST_AS_POSSIBLE 0
  if(timing_mode)
    {
      timer = new Timer(model,mainwindow,AS_FAST_AS_POSSIBLE);
      timer->setTickLimit(TICK_LIMIT);
    }
  else
    {
      timer = new Timer(model,mainwindow,delay_ms);
      mainwindow.show();

    }
  if(!silent)
    cout << "Demo setup complete, running ..." << endl;
  int retval = 0;
  std::chrono::time_point<std::chrono::system_clock> start,stop;
  start = std::chrono::system_clock::now();

  // If timing mode, just go as fast as possible without delays or graphical updates
  if(timing_mode)
  {
    timer->busyTick();
  }
  else
  {
    timer->qtTimerTick();
    retval = app.exec();
  }

  stop = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = stop-start;
  if(!silent)
    cout << "Time: " << elapsed_seconds.count() << " seconds." << endl;
  if(plot)
    {
      std::fstream file;
      file.open("./testdata.txt", std::fstream::out | std::fstream::app);
      file << "Implementation: " << ((IMP == Ped::IMPLEMENTATION::SEQ) ? "Sequential" : (IMP == Ped::IMPLEMENTATION::OMP) ? "OpenMP  " : "PThreads") << "\t Threads: " << threads << "\t Time: " << elapsed_seconds.count() << " seconds." << endl;
      file.close();
    }
  if(!silent)
    cout << "Done" << endl;
  delete (timer);
  return retval;
}
