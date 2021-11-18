#pragma once

#ifndef CODEPOOL_H
#define CODEPOOL_H
#include <string>
int test();

void create_write(void);

void read_pcd(void);

void show_PCD(std::string path);
void show_PCD_2(std::string path);

void reconstruction(std::string path);

void trangulation(std::string path);

int triangle(std::string pcd_file);

#endif // !CODEPOOL_H

