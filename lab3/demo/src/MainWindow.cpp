#include "MainWindow.h"

#include <QtGui>
#include <QBrush>

#include <iostream>
MainWindow::MainWindow(const Ped::Model &model) : model(model)
{
  
  // The Window 
  graphicsView = new QGraphicsView();

  setCentralWidget(graphicsView);

  // A surface for managing a large number of 2D graphical items
  scene = new QGraphicsScene(QRect(0,0,800,600),this);
  
  // Connect
  graphicsView->setScene(scene);


  // Paint on surface
  scene->setBackgroundBrush(Qt::white);
  //graphicsscene->setItemIndexMethod(QGraphicsScene::NoIndex);
  
  if(!model.getDrawTree())
  {
   for (int x=0; x<=800; x+=cellsizePixel)
   {
     scene->addLine(x,0,x,800, QPen(Qt::black));
   }

// // Now add the horizontal lines, paint them green
   for (int y=0; y<=800; y+=cellsizePixel)
   {
     scene->addLine(0,y,800,y, QPen(Qt::black));
   }
  }
  // Create viewAgents with references to the position of the model counterparts
  auto &agents = model.getAgents();
  for(auto agent : agents)
    {
      viewAgents.push_back(new ViewAgent(agent,scene));
    }

  paint();
  graphicsView->show(); // Redundant? 
}

 
void MainWindow::paint() {
  //std::cout << "painting" << endl;

  if(model.getDrawTree())
  {
    for (unsigned int i = 0; i < trees.size(); i++) {
      scene->removeItem(trees[i]);
	}
	trees.clear();
 
	set<const Ped::Ttree*> leafs = model.tree->getLeafs();
	for (set<const Ped::Ttree*>::iterator it = leafs.begin(); it != leafs.end(); ++it) {
      Ped::Ttree *t = (Ped::Ttree *)(*it);
      trees.push_back(scene->addRect(t->getx() * cellsizePixel,
                                     t->gety() * cellsizePixel,
                                     t->getw() * cellsizePixel,
                                     t->geth() * cellsizePixel));
      QString x = QString("%1").arg(t->agents.size());
      QGraphicsSimpleTextItem *y = scene->addSimpleText(x);
      y->setPos((t->getx() + t->getw() / 2) * cellsizePixel,
                (t->gety() + t->geth() / 2) * cellsizePixel);
      trees.push_back(y);
 
	}
  } 
  
  for(auto a : viewAgents)
    {
      a->paint();
    }
}

  
int MainWindow::cellToPixel(int val)
{
  //cout << "conv" << val << " " << val*cellsizePixel << endl;
  return val*cellsizePixel;
}
