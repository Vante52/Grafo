#include <iostream>
#include "Grafo.h" 

// Declaración de funciones auxiliares
template <class T>
void mostrarListaAdyacencia(const Grafo<T>& grafo);
template <class T>
void mostrarRutas(const std::vector<std::vector<unsigned long>>& rutas, const Grafo<T>& grafo);
void pruebaNumeros();
void pruebaLetras();
void pruebaDijkstra();
void pruebaPrim();

int main() {
    //pruebaPrim();
    //pruebaDijkstra();
    //pruebaNumeros();
    //pruebaLetras();
    return 0;
}

// Función para mostrar la lista de adyacencia
template <class T>
void mostrarListaAdyacencia(const Grafo<T>& grafo) {
    std::cout << "Lista de Adyacencia:" << std::endl;
    const std::vector<T>& vertices = grafo.getVertices();
    const std::vector<std::map<int, float>>& aristas = grafo.getAristas();
    for (size_t i = 0; i < aristas.size(); ++i) {
        std::cout << vertices[i] << ": ";
        for (const std::pair<int, float>& arista : aristas[i]) {
            int dest = arista.first;
            float cost = arista.second;
            std::cout << "(" << vertices[dest] << ", " << cost << ") ";
        }
        std::cout << std::endl;
    }
}

// Función para mostrar las rutas resultantes de Prim o Dijkstra
template <class T>
void mostrarRutas(const std::vector<std::vector<unsigned long>>& rutas, const Grafo<T>& grafo) {
    const std::vector<T>& vertices = grafo.getVertices();
    for (size_t i = 0; i < rutas.size(); ++i) {
        if (!rutas[i].empty()) {
            std::cout << "Ruta a " << vertices[rutas[i].back()] << ": ";
            for (unsigned long vertice : rutas[i]) {
                std::cout << vertices[vertice] << " ";
            }
            std::cout << std::endl;
        }
    }
}

// Prueba con vértices numéricos
void pruebaNumeros() {
    // Crear un grafo de tipo int
    Grafo<int> grafo;

    // Insertar vértices
    std::cout << "Insertando vértices..." << std::endl;
    grafo.insertarVertice(1);
    grafo.insertarVertice(2);
    grafo.insertarVertice(3);
    grafo.insertarVertice(4);

    // Insertar aristas
    std::cout << "Insertando aristas..." << std::endl;
    grafo.insertarArista(1, 2, 10);
    grafo.insertarArista(1, 3, 15);
    grafo.insertarArista(2, 4, 20);
    grafo.insertarArista(3, 4, 25);

    // Mostrar la lista de adyacencia
    mostrarListaAdyacencia(grafo);

    // Obtener y mostrar vecinos del vértice 1
    std::vector<int> vecinos = grafo.vecinosVertice(1);
    std::cout << "Vecinos del vértice 1: ";
    for (int v : vecinos) {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    // Realizar y mostrar recorrido en profundidad (DFS)
    std::cout << "Recorrido en profundidad (DFS): ";
    std::vector<int> dfs = grafo.DFS(1);
    for (int v : dfs) {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    // Realizar y mostrar recorrido en anchura (BFS)
    std::cout << "Recorrido en anchura (BFS): ";
    std::vector<int> bfs = grafo.BFS(1);
    for (int v : bfs) {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    // Mostrar las aristas desde el vértice 1
    std::cout << "Aristas desde vértice 1:" << std::endl;
    for (int i = 0; i < grafo.cantVertices(); ++i) {
        int peso = grafo.buscarArista(1, grafo.getVertices()[i]);
        if (peso != -1 && grafo.getVertices()[i] != 1) { // Excluir aristas no existentes y bucle
            std::cout << "  Vértice 1 a Vértice " << grafo.getVertices()[i] << " con peso " << peso << std::endl;
        }
    }

    // Eliminar un vértice
    std::cout << "Eliminando vértice 2..." << std::endl;
    if (grafo.eliminarVertice(2)) {
        std::cout << "Vértice 2 eliminado." << std::endl;
    } else {
        std::cout << "Vértice 2 no encontrado." << std::endl;
    }

    // Mostrar la lista de adyacencia después de la eliminación
    std::cout << "Lista de Adyacencia después de la eliminación:" << std::endl;
    mostrarListaAdyacencia(grafo);

    // Mostrar los vértices después de la eliminación
    std::cout << "Vértices después de la eliminación: ";
    const std::vector<int>& vertsAfter = grafo.getVertices();
    for (int v : vertsAfter) {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    // Realizar y mostrar recorridos nuevamente
    std::cout << "Recorrido en profundidad (DFS): ";
    dfs = grafo.DFS(1);
    for (int v : dfs) {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    std::cout << "Recorrido en anchura (BFS): ";
    bfs = grafo.BFS(1);
    for (int v : bfs) {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    // Insertar nuevas aristas entre los vértices restantes
    std::cout << "Insertando nuevas aristas entre los vértices restantes..." << std::endl;
    grafo.insertarArista(1, 4, 30);
    grafo.insertarArista(3, 1, 5);

    // Mostrar la lista de adyacencia después de insertar nuevas aristas
    mostrarListaAdyacencia(grafo);

    // Realizar y mostrar recorridos nuevamente
    std::cout << "Recorrido en profundidad (DFS): ";
    dfs = grafo.DFS(1);
    for (int v : dfs) {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    std::cout << "Recorrido en anchura (BFS): ";
    bfs = grafo.BFS(1);
    for (int v : bfs) {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    // Eliminar una arista
    std::cout << "Eliminando la arista entre 1 y 3..." << std::endl;
    if (grafo.eliminarArista(1, 3)) {
        std::cout << "Arista entre 1 y 3 eliminada." << std::endl;
    } else {
        std::cout << "Arista entre 1 y 3 no encontrada." << std::endl;
    }

    // Mostrar las aristas desde el vértice 1 después de la eliminación
    std::cout << "Aristas desde vértice 1 después de la eliminación:" << std::endl;
    for (int i = 0; i < grafo.cantVertices(); ++i) {
        int peso = grafo.buscarArista(1, grafo.getVertices()[i]);
        if (peso != -1 && grafo.getVertices()[i] != 1) { // Excluir aristas no existentes y bucle
            std::cout << "  Vértice 1 a Vértice " << grafo.getVertices()[i] << " con peso " << peso << std::endl;
        }
    }

    // Eliminar todos los vértices uno por uno
    std::cout << "Eliminando todos los vértices uno por uno..." << std::endl;
    std::vector<int> verticesAEliminar = grafo.getVertices(); // Copiar los vértices actuales
    for (int vertice : verticesAEliminar) {
        std::cout << "Eliminando vértice " << vertice << "..." << std::endl;
        grafo.eliminarVertice(vertice);
        mostrarListaAdyacencia(grafo);
    }
}

// Prueba con vértices de tipo char
void pruebaLetras() {
    // Crear un grafo de tipo char
    Grafo<char> grafo;

    // Insertar vértices
    std::cout << "Insertando vértices..." << std::endl;
    grafo.insertarVertice('a');
    grafo.insertarVertice('b');
    grafo.insertarVertice('c');
    grafo.insertarVertice('d');
    grafo.insertarVertice('e');
    grafo.insertarVertice('f');

    // Insertar aristas no dirigidas
    std::cout << "Insertando aristas..." << std::endl;
    grafo.insAristaNoDir('a', 'b', 1);
    grafo.insAristaNoDir('a', 'c', 1);
    grafo.insAristaNoDir('a', 'd', 1);
    grafo.insAristaNoDir('b', 'c', 1);
    grafo.insAristaNoDir('b', 'e', 1);
    grafo.insAristaNoDir('d', 'e', 1);
    grafo.insAristaNoDir('d', 'f', 1);

    // Mostrar la lista de adyacencia
    mostrarListaAdyacencia(grafo);

    // Realizar y mostrar recorrido en profundidad (DFS) desde cada vértice
    std::cout << "Recorrido en profundidad (DFS):" << std::endl;
    const std::vector<char>& vertices = grafo.getVertices();
    for (char vertice : vertices) {
        std::cout << "Recorrido desde " << vertice << ": ";
        std::vector<char> dfs = grafo.DFS(vertice);
        for (char v : dfs) {
            std::cout << v << " ";
        }
        std::cout << std::endl;
    }

    // Realizar y mostrar recorrido en anchura (BFS) desde cada vértice
    std::cout << "Recorrido en anchura (BFS):" << std::endl;
    for (char vertice : vertices) {
        std::cout << "Recorrido desde " << vertice << ": ";
        std::vector<char> bfs = grafo.BFS(vertice);
        for (char v : bfs) {
            std::cout << v << " ";
        }
        std::cout << std::endl;
    }

    // Mostrar la lista de adyacencia antes de eliminar aristas
    std::cout << "Antes de eliminar Arista:" << std::endl;
    mostrarListaAdyacencia(grafo);

    // Eliminar aristas no dirigidas
    std::cout << "Eliminando aristas entre 'a' y 'b', y 'a' y 'c'..." << std::endl;
    grafo.elimAristaNoDir('a', 'b');
    grafo.elimAristaNoDir('a', 'c');

    // Mostrar la lista de adyacencia después de eliminar aristas
    std::cout << "Después de eliminar Aristas:" << std::endl;
    mostrarListaAdyacencia(grafo);
}

// Prueba del algoritmo de Dijkstra
void pruebaDijkstra() {
    // Crear un grafo de tipo int
    Grafo<int> grafo;

    // Insertar vértices
    std::cout << "Insertando vértices para Dijkstra..." << std::endl;
    grafo.insertarVertice(0);
    grafo.insertarVertice(1);
    grafo.insertarVertice(2);
    grafo.insertarVertice(3);

    // Insertar aristas
    std::cout << "Insertando aristas para Dijkstra..." << std::endl;
    grafo.insertarArista(0, 1, 4);
    grafo.insertarArista(0, 2, 1);
    grafo.insertarArista(2, 1, 2);
    grafo.insertarArista(1, 3, 1);
    grafo.insertarArista(2, 3, 5);

    // Mostrar la lista de adyacencia
    mostrarListaAdyacencia(grafo);

    // Ejecutar el algoritmo de Dijkstra
    std::cout << "Ejecutando Dijkstra desde el nodo fuente 0..." << std::endl;
    unsigned long i_fuente = 0;
    std::vector<std::vector<unsigned long>> todasRutas = grafo.dijkstra(i_fuente);

    // Mostrar las rutas resultantes
    std::cout << "Árbol de expansión mínima (Dijkstra) - Rutas desde el vértice " << i_fuente << ":" << std::endl;
    mostrarRutas(todasRutas, grafo);
}

// Prueba del algoritmo de Prim
void pruebaPrim() {
    // Crear un grafo de tipo int
    Grafo<int> grafo;

    // Insertar vértices
    std::cout << "Insertando vértices para Prim..." << std::endl;
    grafo.insertarVertice(0);
    grafo.insertarVertice(1);
    grafo.insertarVertice(2);
    grafo.insertarVertice(3);
    grafo.insertarVertice(4);

    // Insertar aristas
    std::cout << "Insertando aristas para Prim..." << std::endl;
    grafo.insertarArista(0, 1, 2);
    grafo.insertarArista(0, 3, 6);
    grafo.insertarArista(1, 2, 3);
    grafo.insertarArista(1, 3, 8);
    grafo.insertarArista(1, 4, 5);
    grafo.insertarArista(2, 4, 7);
    grafo.insertarArista(3, 4, 9);

    // Mostrar la lista de adyacencia
    mostrarListaAdyacencia(grafo);

    // Ejecutar el algoritmo de Prim
    std::cout << "Ejecutando Prim desde el nodo fuente 0..." << std::endl;
    int indiceInicio = 0;
    std::vector<std::vector<unsigned long>> todasRutas = grafo.prim(indiceInicio);

    // Mostrar las rutas resultantes
    std::cout << "Árbol de expansión mínima (Prim) - Rutas desde el vértice " << indiceInicio << ":" << std::endl;
    mostrarRutas(todasRutas, grafo);
}
