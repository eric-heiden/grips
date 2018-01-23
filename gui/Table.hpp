#pragma once

#include <QtCore/QAbstractTableModel>
#include <QtCore/QRect>
#include <QtWidgets/QTableView>
#include <QtCharts/QVXYModelMapper>
#include <QtWidgets/QHeaderView>

#include <sstream>
#include <iomanip>

#include "base/PathStatistics.hpp"


class Table : public QAbstractTableModel
{
public:
    explicit Table(QObject *parent = nullptr)
    : QAbstractTableModel(parent)
    {}

    std::vector<std::string> header;
    std::vector<std::pair<std::string, std::vector<double> > > rows;

    QWidget *createWidget()
    {
        auto *tableView = new QTableView;
        tableView->setModel(this);
        tableView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        tableView->verticalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        tableView->show();
        return tableView;
    }

    std::string str()
    {
        std::vector<int> columnSizes(header.size()+1, 0);
        for (auto &row : rows)
            columnSizes[0] = std::max((int) row.first.length(), columnSizes[0]);
        for (int i = 0; i < header.size(); ++i)
            columnSizes[i+1] = std::max((int) header[i].length(), 15);
        std::stringstream ss;
        ss << std::setw(columnSizes[0]) << " ";
        int c = 1;
        for (auto &h : header)
            ss << std::setw(columnSizes[c++]) << h;
        ss << std::endl;
        for (auto &row : rows)
        {
            ss << std::setw(columnSizes[0]) << row.first;
            c = 1;
            for (auto d : row.second)
                ss << std::setw(columnSizes[c++]) << std::setprecision(10) << std::right << d;
            ss << std::endl;
        }
        return ss.str();
    }

    std::string latex()
    {
        std::vector<int> columnSizes(header.size()+1, 0);
        for (auto &row : rows)
            columnSizes[0] = std::max((int) row.first.length(), columnSizes[0]);
        for (int i = 0; i < header.size(); ++i)
            columnSizes[i+1] = std::max((int) header[i].length(), 20);
        std::stringstream ss;
        ss << std::setw(columnSizes[0]);
        int c = 1;
        for (auto &h : header)
            ss  << " & " << std::setw(columnSizes[c++]) << h;
        ss << " \\\\\\hline" << std::endl;
        for (auto &row : rows)
        {
            ss << std::setw(columnSizes[0]) << row.first;
            c = 1;
            double m = stat::min(row.second);
            for (auto d : row.second)
            {
                ss << " & " << std::setw(columnSizes[c++]);
                if (d == m)
                    ss << "\\bfseries ";
                ss << std::setprecision(4) << std::right << d;
            }
            ss << " \\\\" << std::endl;
        }
        return ss.str();
    }

    int rowCount(const QModelIndex &parent = QModelIndex()) const override
    {
        return static_cast<int>(rows.size() + 1);
    }

    int columnCount(const QModelIndex &parent = QModelIndex()) const override
    {
        return static_cast<int>(header.size());
    }

    QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override
    {
        if (orientation == Qt::Horizontal)
            return QString::fromStdString(header[section % header.size()]);
        return QString::fromStdString(rows[section % rows.size()].first);
    }

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override
    {
        if (index.row() <= rowCount()-1 && index.column() <= columnCount()-1)
            return rows[index.row()].second[index.column()];
        return QVariant();
    }

    Qt::ItemFlags flags(const QModelIndex &index) const override
    {
        return QAbstractItemModel::flags(index) | Qt::ItemIsEditable;
    }

    void addStatsRows(std::string columnName,
                      const std::vector<double> &values1,
                      const std::vector<double> &values2,
                      const std::vector<double> &values3,
                      const std::vector<double> &values4,
                      const std::vector<double> &values5)
    {
        rows.push_back(std::make_pair<std::string, std::vector<double> >(
                "Mdn " + columnName, std::vector<double>{
                        values1.empty() ? std::nan("N/A") : stat::median(values1),
                        values2.empty() ? std::nan("N/A") : stat::median(values2),
                        values3.empty() ? std::nan("N/A") : stat::median(values3),
                        values4.empty() ? std::nan("N/A") : stat::median(values4),
                        values5.empty() ? std::nan("N/A") : stat::median(values5)
                }));
        rows.push_back(std::make_pair<std::string, std::vector<double> >(
                "Avg " + columnName, std::vector<double>{
                        values1.empty() ? std::nan("N/A") : stat::mean(values1),
                        values2.empty() ? std::nan("N/A") : stat::mean(values2),
                        values3.empty() ? std::nan("N/A") : stat::mean(values3),
                        values4.empty() ? std::nan("N/A") : stat::mean(values4),
                        values5.empty() ? std::nan("N/A") : stat::mean(values5)
                }));
        rows.push_back(std::make_pair<std::string, std::vector<double> >(
                "Min " + columnName, std::vector<double>{
                        values1.empty() ? std::nan("N/A") : stat::min(values1),
                        values2.empty() ? std::nan("N/A") : stat::min(values2),
                        values3.empty() ? std::nan("N/A") : stat::min(values3),
                        values4.empty() ? std::nan("N/A") : stat::min(values4),
                        values5.empty() ? std::nan("N/A") : stat::min(values5)
                }));
        rows.push_back(std::make_pair<std::string, std::vector<double> >(
                "Max " + columnName, std::vector<double>{
                        values1.empty() ? std::nan("N/A") : stat::max(values1),
                        values2.empty() ? std::nan("N/A") : stat::max(values2),
                        values3.empty() ? std::nan("N/A") : stat::max(values3),
                        values4.empty() ? std::nan("N/A") : stat::max(values4),
                        values5.empty() ? std::nan("N/A") : stat::max(values5)
                }));
        rows.push_back(std::make_pair<std::string, std::vector<double> >(
                "Std " + columnName, std::vector<double>{
                        values1.empty() ? std::nan("N/A") : stat::std(values1),
                        values2.empty() ? std::nan("N/A") : stat::std(values2),
                        values3.empty() ? std::nan("N/A") : stat::std(values3),
                        values4.empty() ? std::nan("N/A") : stat::std(values4),
                        values5.empty() ? std::nan("N/A") : stat::std(values5)
                }));
    }
};
