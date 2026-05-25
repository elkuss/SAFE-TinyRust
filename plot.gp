set title "SAFE-TinyRust: Real-Time Distance Monitoring"
set xlabel "Waktu"
set ylabel "Jarak (cm)"
set grid
set yrange [0:150] # Batas atas tampilan grafik, sesuaikan kebutuhan

# Membaca data yang dialirkan dari serial port ke file teks log
plot "data_jarak.txt" using 1:2 with linespoints linecolor rgb "blue" title "Jarak Real-Time"

pause 0.5
reread