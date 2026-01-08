#include "MainWindow.h"
#include <QProgressDialog>
#include <QApplication>

namespace rbf {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , cloud_(new pcl::PointCloud<pcl::PointXYZ>)
    , cloudLoaded_(false)
    , labelsLoaded_(false)
{
    setupUI();
}

void MainWindow::setupUI() {
    setWindowTitle("RBF 隐式边界重建");
    resize(1024, 768);

    // 创建中央部件
    QWidget* centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    auto* mainLayout = new QHBoxLayout(centralWidget);

    // 左侧控制面板
    auto* controlPanel = new QWidget;
    controlPanel->setFixedWidth(250);
    auto* controlLayout = new QVBoxLayout(controlPanel);

    // 标题
    auto* titleLabel = new QLabel("<h2>RBF 隐式边界重建</h2>");
    titleLabel->setWordWrap(true);
    controlLayout->addWidget(titleLabel);

    controlLayout->addSpacing(20);

    // 按钮
    btnLoadCloud_ = new QPushButton("加载点云");
    btnLoadLabels_ = new QPushButton("加载标签");
    btnRun_ = new QPushButton("运行重建");
    btnClear_ = new QPushButton("清除");

    btnLoadCloud_->setEnabled(true);
    btnLoadLabels_->setEnabled(false);
    btnRun_->setEnabled(false);
    btnClear_->setEnabled(true);

    controlLayout->addWidget(btnLoadCloud_);
    controlLayout->addWidget(btnLoadLabels_);
    controlLayout->addSpacing(10);
    controlLayout->addWidget(btnRun_);
    controlLayout->addWidget(btnClear_);

    controlLayout->addSpacing(20);

    // 状态标签
    labelStatus_ = new QLabel("状态: 请加载点云数据");
    labelStatus_->setWordWrap(true);
    controlLayout->addWidget(labelStatus_);

    controlLayout->addStretch();

    // 说明
    auto* infoLabel = new QLabel(
        "<b>使用说明:</b><br>"
        "1. 加载点云文件 (PCD/PLY/XYZ)<br>"
        "2. 加载对应的标签文件<br>"
        "3. 点击'运行重建'<br>"
        "4. 查看重建结果"
    );
    infoLabel->setWordWrap(true);
    controlLayout->addWidget(infoLabel);

    // 右侧查看器
    viewer_ = new PointCloudViewer;

    mainLayout->addWidget(controlPanel);
    mainLayout->addWidget(viewer_, 1);

    // 连接信号
    connect(btnLoadCloud_, &QPushButton::clicked, this, &MainWindow::onLoadPointCloud);
    connect(btnLoadLabels_, &QPushButton::clicked, this, &MainWindow::onLoadLabels);
    connect(btnRun_, &QPushButton::clicked, this, &MainWindow::onRunReconstruction);
    connect(btnClear_, &QPushButton::clicked, this, &MainWindow::onClear);
}

void MainWindow::onLoadPointCloud() {
    QString filename = QFileDialog::getOpenFileName(
        this,
        "选择点云文件",
        "",
        "点云文件 (*.pcd *.ply *.xyz);;所有文件 (*.*)"
    );

    if (filename.isEmpty()) {
        return;
    }

    try {
        cloud_ = PointCloudLoader::load(filename.toStdString());
        cloudLoaded_ = true;

        // 显示点云
        viewer_->showPointCloud(cloud_, "input_cloud");
        labelStatus_->setText(
            QString("状态: 已加载点云 (%1 个点)\n请加载标签文件").arg(cloud_->size())
        );

        btnLoadLabels_->setEnabled(true);
        btnRun_->setEnabled(false);

    } catch (const std::exception& e) {
        QMessageBox::critical(this, "错误", QString("加载点云失败:\n%1").arg(e.what()));
    }
}

void MainWindow::onLoadLabels() {
    QString filename = QFileDialog::getOpenFileName(
        this,
        "选择标签文件",
        "",
        "标签文件 (*.txt *.labels);;所有文件 (*.*)"
    );

    if (filename.isEmpty()) {
        return;
    }

    try {
        labels_ = PointCloudLoader::loadLabels(filename.toStdString());

        if (labels_.size() != cloud_->size()) {
            QMessageBox::warning(
                this,
                "警告",
                QString("标签数量 (%1) 与点云点数 (%2) 不匹配")
                    .arg(labels_.size())
                    .arg(cloud_->size())
            );
            return;
        }

        labelsLoaded_ = true;
        labelStatus_->setText(
            QString("状态: 已加载点云 (%1 点) 和标签\n可以运行重建").arg(cloud_->size())
        );

        btnRun_->setEnabled(true);

    } catch (const std::exception& e) {
        QMessageBox::critical(this, "错误", QString("加载标签失败:\n%1").arg(e.what()));
    }
}

void MainWindow::onRunReconstruction() {
    if (!haveData()) {
        return;
    }

    // 显示进度对话框
    QProgressDialog progress("正在进行 RBF 隐式边界重建...", "取消", 0, 4, this);
    progress.setWindowTitle("重建中");
    progress.setWindowModality(Qt::WindowModal);
    progress.show();

    try {
        // 步骤 1: 计算符号距离函数
        progress.setValue(1);
        progress.setLabelText("计算符号距离函数...");
        QApplication::processEvents();

        std::vector<double> distances = DistanceFunction::compute(cloud_, labels_);

        // 步骤 2: 构建 RBF 插值器
        progress.setValue(2);
        progress.setLabelText("构建 RBF 插值器...");
        QApplication::processEvents();

        auto interpolator = std::make_shared<RBFInterpolator>(cloud_, distances, 0.1);

        if (!interpolator->solve()) {
            QMessageBox::critical(this, "错误", "RBF 线性系统求解失败");
            return;
        }

        // 步骤 3: Marching Cubes 提取边界
        progress.setValue(3);
        progress.setLabelText("提取边界网格...");
        QApplication::processEvents();

        MarchingCubes mc(interpolator, 30);
        pcl::PolygonMesh mesh = mc.extract();

        // 步骤 4: 显示结果
        progress.setValue(4);
        progress.setLabelText("完成!");

        viewer_->clearAll();
        viewer_->showPointCloud(cloud_, "cloud");
        viewer_->showMesh(std::make_shared<pcl::PolygonMesh>(mesh), "mesh");

        labelStatus_->setText(
            QString("状态: 重建完成!\n点云: %1 点\n网格: %2 三角形")
                .arg(cloud_->size())
                .arg(mesh.polygons.size())
        );

    } catch (const std::exception& e) {
        QMessageBox::critical(this, "错误", QString("重建失败:\n%1").arg(e.what()));
        labelStatus_->setText("状态: 重建失败");
    }
}

void MainWindow::onClear() {
    viewer_->clearAll();
    cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    labels_.clear();
    cloudLoaded_ = false;
    labelsLoaded_ = false;

    btnLoadLabels_->setEnabled(false);
    btnRun_->setEnabled(false);
    labelStatus_->setText("状态: 已清除，请重新加载数据");
}

bool MainWindow::haveData() const {
    return cloudLoaded_ && labelsLoaded_ && !cloud_->empty();
}

} // namespace rbf
