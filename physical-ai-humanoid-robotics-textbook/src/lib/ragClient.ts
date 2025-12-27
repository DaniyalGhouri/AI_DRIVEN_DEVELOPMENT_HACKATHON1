// physical-ai-humanoid-robotics-textbook/src/lib/ragClient.ts

import axios from 'axios';

const BACKEND_URL = "http://localhost:8000";

interface FileContent {
  file_path: string;
  content: string;
  module_id: string;
  chapter_id?: string;
  section_id?: string;
  page_number?: number;
}

interface IngestRequest {
  files: FileContent[];
}

interface QueryRequest {
  query: string;
  user_id?: string;
  module_context?: string;
}

interface HighlightedQueryRequest {
  query: string;
  highlighted_text: string;
  user_id?: string;
  module_context?: string;
}

interface TranslationRequest {
  text: string;
  target_language: string;
}

interface ChatResponse {
  answer: string;
  citations: any[]; // Define a more specific type if known
}

interface TranslatedResponse {
  translated_text: string;
}

interface Token {
  access_token: string;
  token_type: string;
}

// Function to get the access token from local storage
const getAccessToken = (): string | null => {
  return localStorage.getItem('access_token');
};

// Interceptor to add authorization header to all requests
axios.interceptors.request.use(
  (config) => {
    const token = getAccessToken();
    if (token) {
      config.headers.Authorization = `Bearer ${token}`;
    }
    return config;
  },
  (error) => {
    return Promise.reject(error);
  }
);

export const ingestContent = async (files: FileContent[]): Promise<any> => {
  try {
    const response = await axios.post(`${BACKEND_URL}/ingest`, { files });
    return response.data;
  } catch (error) {
    console.error("Error ingesting content:", error);
    throw error;
  }
};

export const queryRAG = async (query: string, user_id?: string, module_context?: string): Promise<ChatResponse> => {
  try {
    const response = await axios.post(`${BACKEND_URL}/query`, {
      query,
      user_id,
      module_context
    });
    return response.data;
  } catch (error) {
    console.error("Error querying RAG:", error);
    throw error;
  }
};

export const highlightedQueryRAG = async (query: string, highlighted_text: string, user_id?: string, module_context?: string): Promise<ChatResponse> => {
  try {
    const response = await axios.post(`${BACKEND_URL}/highlighted-query`, {
      query,
      highlighted_text,
      user_id,
      module_context
    });
    return response.data;
  } catch (error) {
    console.error("Error querying RAG with highlighted text:", error);
    throw error;
  }
};

export const translateText = async (text: string, target_language: string): Promise<TranslatedResponse> => {
  try {
    const response = await axios.post(`${BACKEND_URL}/translate`, {
      text,
      target_language
    });
    return response.data;
  } catch (error) {
    console.error("Error translating text:", error);
    throw error;
  }
};

export const getRecommendations = async (): Promise<{ recommendations: string }> => {
  try {
    const response = await axios.get(`${BACKEND_URL}/users/me/recommendations`);
    return response.data;
  } catch (error) {
    console.error("Error getting recommendations:", error);
    throw error;
  }
};

export const personalizeContent = async (text: string, module_context?: string): Promise<{ personalized_text: string }> => {
  try {
    const response = await axios.post(`${BACKEND_URL}/personalize`, {
      text,
      module_context
    });
    return response.data;
  } catch (error) {
    console.error("Error personalizing content:", error);
    throw error;
  }
};

// Authentication functions
export const registerUser = async (username: string, password: string, email?: string, software_background?: string, hardware_background?: string) => {
  try {
    const response = await axios.post(`${BACKEND_URL}/register`, {
      username,
      password,
      email,
      software_background,
      hardware_background
    });
    return response.data;
  } catch (error) {
    console.error("Error registering user:", error);
    throw error;
  }
};

export const loginUser = async (username: string, password: string): Promise<Token> => {
  try {
    // Create form data for the login request
    const formData = new FormData();
    formData.append('username', username);
    formData.append('password', password);

    const response = await axios.post(`${BACKEND_URL}/token`, formData, {
      headers: {
        'Content-Type': 'application/x-www-form-urlencoded',
      }
    });
    return response.data;
  } catch (error) {
    console.error("Error logging in:", error);
    throw error;
  }
};

export const getUserProfile = async () => {
  try {
    const response = await axios.get(`${BACKEND_URL}/users/me/`);
    return response.data;
  } catch (error) {
    console.error("Error getting user profile:", error);
    throw error;
  }
};

export const getUserNotes = async () => {
  try {
    const response = await axios.get(`${BACKEND_URL}/users/me/notes`);
    return response.data;
  } catch (error) {
    console.error("Error getting user notes:", error);
    throw error;
  }
};

export const saveNote = async (content: string, module_id?: string, text_selection?: string) => {
  try {
    const response = await axios.post(`${BACKEND_URL}/users/me/notes`, {
      note_content: content,
      module_id,
      text_selection
    });
    return response.data;
  } catch (error) {
    console.error("Error saving note:", error);
    throw error;
  }
};

export const getUserChatSessions = async () => {
  try {
    const response = await axios.get(`${BACKEND_URL}/users/me/chat_sessions`);
    return response.data;
  } catch (error) {
    console.error("Error getting chat sessions:", error);
    throw error;
  }
};

// Health check
export const healthCheck = async () => {
  try {
    const response = await axios.get(`${BACKEND_URL}/health`);
    return response.data;
  } catch (error) {
    console.error("Error checking health:", error);
    throw error;
  }
};