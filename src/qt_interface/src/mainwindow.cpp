#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    QMessageBox::information(this, "Message", "You clicked the button!");
}



// #include "mainwindow.h"

// MainWindow::MainWindow(QWidget *parent)
//     : QMainWindow(parent)
// {
//     // Set the size of the window
//     setFixedSize(400, 300);

//     // Create a new push button
//     QPushButton *button = new QPushButton("Click me", this);
//     button->setGeometry(QRect(QPoint(100, 100), QSize(200, 50)));

//     // Connect the clicked signal of the button to the onButtonClicked slot
//     connect(button, &QPushButton::clicked, this, &MainWindow::onButtonClicked);
// }

// void MainWindow::onButtonClicked()
// {
//     // Show a message box when the button is clicked
//     QMessageBox::information(this, "Hello!", "You clicked the button!");
// }



void MainWindow::on_pushButton_2_clicked(bool checked)
{

}

